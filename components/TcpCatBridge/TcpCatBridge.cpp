#include "TcpCatBridge.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <algorithm>
#include <cstring>
#include <utility>
#include <functional>
#include <mutex>

namespace tcp_cat_bridge {

namespace {

// Match USB CDC sanitization: control chars (0x00-0x1F) except ';' are stripped
bool isControlChar(unsigned char c) {
    return c <= 0x1F && c != ';';
}

} // namespace

TcpCatBridge::TcpCatBridge(uint16_t port, uint8_t bridgeId)
    : bridgeId_(bridgeId)
    , serverPort_(port) {

    // Validate bridge ID
    if (bridgeId > 1) {
        ESP_LOGE(TAG, "Invalid bridge ID %d (must be 0 or 1)", bridgeId);
    }
}

TcpCatBridge::~TcpCatBridge() {
    stop();
}

void TcpCatBridge::setIncomingFrameCallback(std::function<void(std::string_view)> callback) {
    RtosLockGuard<RtosMutex> lock(callbackMutex_);
    incomingFrameCallback_ = std::move(callback);
}

esp_err_t TcpCatBridge::start() {
    if (running_.load()) {
        ESP_LOGW(TAG, "Bridge %d already running", bridgeId_);
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize server socket
    esp_err_t err = initSocket();
    if (err != ESP_OK) {
        return err;
    }

    running_.store(true);
    taskExited_.store(false);

    const uint32_t stackDepth = std::max<int>(CONFIG_TCP_CAT_BRIDGE_TASK_STACK_SIZE, MIN_STACK_DEPTH);

    // Create bridge task with unique name
    char taskName[16];
    snprintf(taskName, sizeof(taskName), "tcp_cat%d", bridgeId_);

    BaseType_t result = xTaskCreate(
        bridgeTask,
        taskName,  // Unique task name per instance
        stackDepth,
        this,
        CONFIG_TCP_CAT_BRIDGE_TASK_PRIORITY,
        &taskHandle_);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create bridge task for bridge %d", bridgeId_);
        cleanup();
        running_.store(false);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TCP-CAT bridge started for bridge %d on port %d", bridgeId_, serverPort_);
    return ESP_OK;
}

void TcpCatBridge::stop() {
    if (!running_.load()) {
        return;
    }

    ESP_LOGI(TAG, "Stopping TCP-CAT bridge for bridge %d", bridgeId_);

    // Signal shutdown. The task owns cleanup(); stop() only joins.
    running_.store(false);

    // Nudge the listening socket so a task parked in select() wakes promptly
    // instead of waiting out the full select timeout. The task's cleanup()
    // still owns the close(), so we only shutdown() here (never close()).
    if (serverSocket_ >= 0) {
        shutdown(serverSocket_, SHUT_RDWR);
    }

    // Join: wait for the task to confirm it exited its loop and ran cleanup().
    // Never force-delete — the task may hold clientsMutex_, and vTaskDelete
    // would strand it (deadlocking the next lock acquisition).
    if (taskHandle_ != nullptr) {
        constexpr int JOIN_TIMEOUT_MS = 2000;
        constexpr int POLL_MS = 10;
        int waited = 0;
        while (!taskExited_.load() && waited < JOIN_TIMEOUT_MS) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            waited += POLL_MS;
        }
        if (!taskExited_.load()) {
            ESP_LOGE(TAG, "Bridge %d task did not exit within %d ms; leaving it (may hold a lock)",
                     bridgeId_, JOIN_TIMEOUT_MS);
        }
        taskHandle_ = nullptr;
    }

    ESP_LOGI(TAG, "TCP-CAT bridge for bridge %d stopped", bridgeId_);
}

esp_err_t TcpCatBridge::initSocket() {
    // Create socket
    serverSocket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverSocket_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        ESP_LOGW(TAG, "Failed to set SO_REUSEADDR: errno %d", errno);
    }

    // Set non-blocking
    int flags = fcntl(serverSocket_, F_GETFL, 0);
    if (fcntl(serverSocket_, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGW(TAG, "Failed to set non-blocking: errno %d", errno);
    }

    // Bind to configured address (default 0.0.0.0 = all interfaces)
    struct sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort_);

    const char* bindAddr = CONFIG_TCP_CAT_BRIDGE_BIND_ADDR;
    if (bindAddr[0] == '\0' || inet_aton(bindAddr, &serverAddr.sin_addr) == 0) {
        // Empty or unparseable address falls back to all interfaces
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bindAddr[0] != '\0') {
            ESP_LOGW(TAG, "Invalid bind address '%s', using 0.0.0.0", bindAddr);
        }
    }

    if (bind(serverSocket_, reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket to port %d: errno %d", serverPort_, errno);
        close(serverSocket_);
        serverSocket_ = -1;
        return ESP_FAIL;
    }

    // Listen
    if (listen(serverSocket_, MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "Failed to listen on port %d: errno %d", serverPort_, errno);
        close(serverSocket_);
        serverSocket_ = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Server socket listening on port %d for bridge %d", serverPort_, bridgeId_);
    return ESP_OK;
}

int TcpCatBridge::acceptClient() {
    struct sockaddr_in clientAddr{};
    socklen_t addrLen = sizeof(clientAddr);

    int clientSock = accept(serverSocket_, reinterpret_cast<struct sockaddr*>(&clientAddr), &addrLen);
    if (clientSock < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "Accept failed: errno %d", errno);
        }
        return -1;
    }

    RtosLockGuard<RtosMutex> lock(clientsMutex_);

    // Rate limit accepted connections. Only advance the clock on an actual
    // accept, so a tight reconnect loop is throttled (each early attempt is
    // dropped, the real client is not churned). Applies with auth on or off.
    const TickType_t now = xTaskGetTickCount();
    const TickType_t minInterval = pdMS_TO_TICKS(MIN_ACCEPT_INTERVAL_MS);
    if (lastAcceptTick_ != 0 && (now - lastAcceptTick_) < minInterval) {
        ESP_LOGW(TAG, "Bridge %d dropping connection: accept rate limit (< %d ms)",
                 bridgeId_, MIN_ACCEPT_INTERVAL_MS);
        close(clientSock);
        return -1;
    }
    lastAcceptTick_ = now;

    // Auth disabled (Hamlib default): evict the existing client on connect so a
    // stale session is replaced. Auth enabled: keep the current client until the
    // newcomer proves itself (eviction happens in handleAuthFrame on success).
    if (!AUTH_ENABLED) {
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (clients_[i].connected) {
                ESP_LOGI(TAG, "Closing existing client %d to accept new connection on bridge %d", i, bridgeId_);
                closeClientLocked(i);
                break;  // Only one client can be connected at a time
            }
        }
    }

    // Find a free slot. With auth enabled an authenticated client keeps its slot,
    // so a newcomer needs a spare slot to run the handshake; if none, reject it.
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (!clients_[i].connected) {
            clients_[i].socket = clientSock;
            clients_[i].connected = true;
            clients_[i].authenticated = !AUTH_ENABLED;  // trusted immediately when auth is off
            clients_[i].bytesRx = 0;
            clients_[i].bytesTx = 0;
            clients_[i].pendingLen = 0;
            clients_[i].txPending = 0;

            // Disable Nagle's algorithm to minimize latency for small CAT frames
            int nodelay = 1;
            if (setsockopt(clientSock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay)) < 0) {
                ESP_LOGW(TAG, "Failed to set TCP_NODELAY on client %d: errno %d", i, errno);
            }

            // Set client socket non-blocking
            int flags = fcntl(clientSock, F_GETFL, 0);
            fcntl(clientSock, F_SETFL, flags | O_NONBLOCK);

            clientCount_.fetch_add(1);
            ESP_LOGI(TAG, "Client %d connected to bridge %d from %s:%d",
                     i, bridgeId_, inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
            return i;
        }
    }

    // No free slot. With auth off this is unexpected (we just evicted); with auth
    // on it means an authenticated client already occupies every slot — reject so
    // the current session is not disturbed by an unauthenticated newcomer.
    ESP_LOGW(TAG, "Bridge %d no free slot, rejecting connection", bridgeId_);
    close(clientSock);
    return -1;
}

void TcpCatBridge::handleClientData(int clientIdx, int expectedSock) {
    int sock;
    {
        RtosLockGuard<RtosMutex> lock(clientsMutex_);
        if (clientIdx < 0 || clientIdx >= MAX_CLIENTS || !clients_[clientIdx].connected) {
            return;
        }
        // Revalidate: acceptClient() may have evicted and reused this slot between
        // the select() snapshot and now. Skip if the fd no longer matches — the
        // readiness we observed belonged to the previous occupant of this slot.
        sock = clients_[clientIdx].socket;
        if (sock != expectedSock) {
            return;
        }
    }

    // recv() outside lock — safe because only bridgeTask reads from client sockets
    uint8_t rxBuf[BUFFER_SIZE];
    ssize_t received = recv(sock, rxBuf, BUFFER_SIZE, 0);

    if (received > 0) {
        {
            RtosLockGuard<RtosMutex> lock(clientsMutex_);
            clients_[clientIdx].bytesRx += static_cast<uint64_t>(received);
        }
        bytesReceived_.fetch_add(static_cast<uint64_t>(received));

        ESP_LOGV(TAG, "Bridge %d received %d bytes from client %d", bridgeId_, (int)received, clientIdx);

        // Process input and invoke frame callback for complete CAT commands
        processClientInput(clientIdx, rxBuf, static_cast<size_t>(received));
    } else if (received == 0) {
        // Client disconnected gracefully
        RtosLockGuard<RtosMutex> lock(clientsMutex_);
        ESP_LOGI(TAG, "Client %d disconnected from bridge %d (RX=%llu, TX=%llu)",
                 clientIdx, bridgeId_, clients_[clientIdx].bytesRx, clients_[clientIdx].bytesTx);
        closeClientLocked(clientIdx);
    } else {
        // Error
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "Client %d recv error: errno %d", clientIdx, errno);
            closeClient(clientIdx);
        }
    }
}

void TcpCatBridge::processClientInput(int clientIdx, const uint8_t* data, size_t length) {
    if (clientIdx < 0 || clientIdx >= MAX_CLIENTS || data == nullptr || length == 0) {
        return;
    }

    std::function<void(std::string_view)> frameCallback;
    {
        RtosLockGuard<RtosMutex> lock(callbackMutex_);
        frameCallback = incomingFrameCallback_;
    }

    // Process one frame at a time: lock → extract one complete frame → unlock → dispatch.
    // This prevents ABBA deadlock (clientsMutex_ → dispatchMutex_ vs reverse order)
    // while keeping stack usage minimal (~128 bytes vs ~2KB+ for batching).
    static constexpr size_t MAX_FRAME_LEN = 128; // CAT commands are <=64 bytes; generous limit
    char frameData[MAX_FRAME_LEN];
    size_t inputPos = 0;

    while (inputPos < length) {
        size_t frameLen = 0;
        bool gotFrame = false;
        bool frameAuthed = false;

        {
            RtosLockGuard<RtosMutex> lock(clientsMutex_);
            ClientState& client = clients_[clientIdx];
            if (!client.connected) {
                return;
            }
            frameAuthed = client.authenticated;

            while (inputPos < length) {
                const unsigned char ch = data[inputPos++];

                // Strip all control characters except ';' (matches USB CDC sanitization)
                if (isControlChar(ch)) {
                    continue;
                }

                // Handle buffer overflow: drop oldest data, keep newest (matches USB CDC behavior)
                if (client.pendingLen >= COMMAND_BUFFER_SIZE) {
                    size_t lastDelim = 0;
                    for (size_t j = 0; j < client.pendingLen; ++j) {
                        if (client.pendingBuffer[j] == ';') {
                            lastDelim = j + 1;
                        }
                    }

                    if (lastDelim > 0 && lastDelim < client.pendingLen) {
                        const size_t keep = client.pendingLen - lastDelim;
                        memmove(client.pendingBuffer.data(), client.pendingBuffer.data() + lastDelim, keep);
                        client.pendingLen = keep;
                        ESP_LOGW(TAG, "Client %d buffer overflow: dropped %zu bytes, kept %zu",
                                 clientIdx, lastDelim, keep);
                    } else {
                        const size_t drop = COMMAND_BUFFER_SIZE / 2;
                        const size_t keep = client.pendingLen - drop;
                        memmove(client.pendingBuffer.data(), client.pendingBuffer.data() + drop, keep);
                        client.pendingLen = keep;
                        ESP_LOGW(TAG, "Client %d buffer overflow: dropped %zu bytes, kept %zu",
                                 clientIdx, drop, keep);
                    }
                }

                client.pendingBuffer[client.pendingLen++] = static_cast<char>(ch);

                if (ch == ';') {
                    if (client.pendingLen > MAX_FRAME_LEN) {
                        // Oversized frame (> MAX_FRAME_LEN before terminator): discard
                        // rather than truncate-and-emit a terminator-less frame, which
                        // would desync the client stream. CAT frames are <=64 bytes.
                        ESP_LOGW(TAG, "Client %d oversized frame (%zu B): discarded",
                                 clientIdx, client.pendingLen);
                        client.pendingLen = 0;
                        continue; // keep scanning remaining input for the next frame
                    }
                    memcpy(frameData, client.pendingBuffer.data(), client.pendingLen);
                    frameLen = client.pendingLen;   // <= MAX_FRAME_LEN, no truncation
                    client.pendingLen = 0;
                    gotFrame = true;
                    break; // Exit inner loop to dispatch this frame
                }
            }
        } // clientsMutex_ released here

        // Dispatch WITHOUT holding clientsMutex_
        if (gotFrame) {
            std::string_view frame(frameData, frameLen);
            if (!AUTH_ENABLED || frameAuthed) {
                if (frameCallback) {
                    ESP_LOGV(TAG, "Bridge %d RX from client %d: %.*s", bridgeId_, clientIdx,
                             (int)frame.size(), frame.data());
                    frameCallback(frame);
                }
            } else {
                // Auth enabled and client not yet authenticated: the first frame
                // must be the auth handshake. Never forward it to the CAT callback.
                handleAuthFrame(clientIdx, frame);
            }
        }
    }
}

void TcpCatBridge::handleAuthFrame(int clientIdx, std::string_view frame) {
    // Expected exactly: "AUTH <token>;" (control chars already stripped upstream).
    char expected[128];
    const int n = snprintf(expected, sizeof(expected), "AUTH %s;", CONFIG_TCP_CAT_BRIDGE_AUTH_TOKEN);

    const bool ok = (n > 0 && static_cast<size_t>(n) < sizeof(expected) &&
                     frame.size() == static_cast<size_t>(n) &&
                     memcmp(frame.data(), expected, static_cast<size_t>(n)) == 0);

    if (!ok) {
        ESP_LOGW(TAG, "Bridge %d client %d failed auth, closing", bridgeId_, clientIdx);
        closeClient(clientIdx);
        return;
    }

    {
        RtosLockGuard<RtosMutex> lock(clientsMutex_);
        if (clientIdx < 0 || clientIdx >= MAX_CLIENTS || !clients_[clientIdx].connected) {
            return;
        }
        clients_[clientIdx].authenticated = true;
        // Now enforce single-client: evict any other (previously authenticated) client.
        for (int j = 0; j < MAX_CLIENTS; ++j) {
            if (j != clientIdx && clients_[j].connected) {
                ESP_LOGI(TAG, "Bridge %d evicting client %d in favour of newly authenticated client %d",
                         bridgeId_, j, clientIdx);
                closeClientLocked(j);
            }
        }
    }

    ESP_LOGI(TAG, "Bridge %d client %d authenticated", bridgeId_, clientIdx);
    sendToActiveClient("AUTH1;");
}

void TcpCatBridge::sendToActiveClient(std::string_view message) {
    if (message.empty()) {
        return;
    }

    RtosLockGuard<RtosMutex> lock(clientsMutex_);

    // Send to the active (authenticated) client. Single-client enforcement means
    // at most one client is authenticated; an unauthenticated newcomer (auth mode)
    // never receives radio responses.
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (!clients_[i].connected || !clients_[i].authenticated) {
            continue;
        }

        ESP_LOGV(TAG, "Bridge %d TX to client %d: %.*s", bridgeId_, i,
                 (int)message.size(), message.data());

        // 1. Drain any previously queued remainder first to preserve ordering.
        if (clients_[i].txPending > 0) {
            flushPendingTxLocked(i);
            if (!clients_[i].connected) {
                return;  // closed on hard error
            }
            if (clients_[i].txPending > 0) {
                // Still backed up: queue the new frame behind the pending bytes
                // (whole-frame). Overflow closes the client inside the helper.
                appendPendingTxLocked(i, message.data(), message.size());
                return;
            }
        }

        // 2. Send the new frame directly; stash any would-block remainder.
        const char* ptr = message.data();
        size_t remaining = message.size();
        while (remaining > 0) {
            ssize_t sent = send(clients_[i].socket, ptr, remaining, 0);
            if (sent > 0) {
                clients_[i].bytesTx += static_cast<uint64_t>(sent);
                bytesSent_.fetch_add(static_cast<uint64_t>(sent));
                ptr += sent;
                remaining -= static_cast<size_t>(sent);
            } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "Client %d send error: errno %d", i, errno);
                // clientsMutex_ is already held; use the locked variant to avoid
                // re-taking the non-recursive mutex (deadlock).
                closeClientLocked(i);
                return;
            } else {
                // Would block: queue the unsent remainder so the client never sees
                // a truncated frame. Flushed later (next send / writable select).
                ESP_LOGD(TAG, "Client %d send would block, queuing %zu bytes", i, remaining);
                appendPendingTxLocked(i, ptr, remaining);
                return;
            }
        }
        return;  // Single active client - done
    }

    // No active client - silently drop message
    ESP_LOGV(TAG, "No active client on bridge %d, dropping message: %.*s",
             bridgeId_, static_cast<int>(message.size()), message.data());
}

bool TcpCatBridge::appendPendingTxLocked(int clientIdx, const char* data, size_t length) {
    ClientState& client = clients_[clientIdx];
    if (length == 0) {
        return true;
    }
    if (client.txPending + length > PENDING_TX_SIZE) {
        // Unrecoverable: the client cannot keep up. Closing preserves whole-frame
        // semantics (better a clean disconnect than a permanently desynced stream).
        ESP_LOGW(TAG, "Client %d pending-TX overflow (%zu + %zu > %zu), closing",
                 clientIdx, client.txPending, length, PENDING_TX_SIZE);
        closeClientLocked(clientIdx);
        return false;
    }
    memcpy(client.txBuffer.data() + client.txPending, data, length);
    client.txPending += length;
    return true;
}

void TcpCatBridge::flushPendingTxLocked(int clientIdx) {
    ClientState& client = clients_[clientIdx];
    while (client.txPending > 0) {
        ssize_t sent = send(client.socket, client.txBuffer.data(), client.txPending, 0);
        if (sent > 0) {
            client.bytesTx += static_cast<uint64_t>(sent);
            bytesSent_.fetch_add(static_cast<uint64_t>(sent));
            const size_t rem = client.txPending - static_cast<size_t>(sent);
            if (rem > 0) {
                memmove(client.txBuffer.data(), client.txBuffer.data() + sent, rem);
            }
            client.txPending = rem;
        } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "Client %d pending-TX flush error: errno %d", clientIdx, errno);
            closeClientLocked(clientIdx);
            return;
        } else {
            // Still would block: keep the remainder queued for the next attempt.
            break;
        }
    }
}

void TcpCatBridge::handleClientWritable(int clientIdx, int expectedSock) {
    RtosLockGuard<RtosMutex> lock(clientsMutex_);
    if (clientIdx < 0 || clientIdx >= MAX_CLIENTS || !clients_[clientIdx].connected) {
        return;
    }
    // Revalidate the fd (slot may have been evicted/reused since the snapshot).
    if (clients_[clientIdx].socket != expectedSock) {
        return;
    }
    if (clients_[clientIdx].txPending > 0) {
        flushPendingTxLocked(clientIdx);
    }
}

void TcpCatBridge::closeClientLocked(int clientIdx) {
    if (clientIdx < 0 || clientIdx >= MAX_CLIENTS) {
        return;
    }

    ClientState& client = clients_[clientIdx];
    if (!client.connected) {
        return;
    }

    if (client.socket >= 0) {
        close(client.socket);
        client.socket = -1;
    }

    client.connected = false;
    client.authenticated = false;
    client.pendingLen = 0;
    client.txPending = 0;
    clientCount_.fetch_sub(1);
}

void TcpCatBridge::closeClient(int clientIdx) {
    RtosLockGuard<RtosMutex> lock(clientsMutex_);
    closeClientLocked(clientIdx);
}

void TcpCatBridge::closeAllClients() {
    RtosLockGuard<RtosMutex> lock(clientsMutex_);
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        closeClientLocked(i);
    }
}

void TcpCatBridge::cleanup() {
    closeAllClients();

    if (serverSocket_ >= 0) {
        close(serverSocket_);
        serverSocket_ = -1;
    }

    {
        RtosLockGuard<RtosMutex> lock(callbackMutex_);
        incomingFrameCallback_ = nullptr;
    }
}

void TcpCatBridge::bridgeTask(void* arg) {
    auto* bridge = static_cast<TcpCatBridge*>(arg);

    ESP_LOGI(TAG, "Bridge task started for bridge %d", bridge->bridgeId_);

    fd_set readfds;
    fd_set writefds;
    struct timeval timeout{};

    while (bridge->running_.load()) {
        FD_ZERO(&readfds);
        FD_ZERO(&writefds);

        // Add server socket to set
        int maxfd = bridge->serverSocket_;
        FD_SET(bridge->serverSocket_, &readfds);

        // Snapshot (fd, has-pending) per slot under lock for select() setup.
        // The fd is carried into the handlers so they can revalidate the slot.
        int clientSockets[MAX_CLIENTS];
        {
            RtosLockGuard<RtosMutex> lock(bridge->clientsMutex_);
            for (int i = 0; i < MAX_CLIENTS; ++i) {
                if (bridge->clients_[i].connected) {
                    const int sock = bridge->clients_[i].socket;
                    clientSockets[i] = sock;
                    FD_SET(sock, &readfds);
                    // Only watch for writability while there is queued TX to flush.
                    if (bridge->clients_[i].txPending > 0) {
                        FD_SET(sock, &writefds);
                    }
                    if (sock > maxfd) {
                        maxfd = sock;
                    }
                } else {
                    clientSockets[i] = -1;
                }
            }
        }

        // Set timeout for select
        timeout.tv_sec = 0;
        timeout.tv_usec = SELECT_TIMEOUT_MS * 1000;

        // Wait for activity (outside lock - this blocks)
        int activity = select(maxfd + 1, &readfds, &writefds, nullptr, &timeout);

        if (activity < 0) {
            if (errno != EINTR) {
                ESP_LOGE(TAG, "Select error: errno %d", errno);
                break;
            }
            continue;
        }

        // Check for new client connections
        if (FD_ISSET(bridge->serverSocket_, &readfds)) {
            bridge->acceptClient();
        }

        // Flush queued TX for now-writable clients (fd revalidated inside).
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (clientSockets[i] >= 0 && FD_ISSET(clientSockets[i], &writefds)) {
                bridge->handleClientWritable(i, clientSockets[i]);
            }
        }

        // Check for client data (fd revalidated inside handleClientData).
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (clientSockets[i] >= 0 && FD_ISSET(clientSockets[i], &readfds)) {
                bridge->handleClientData(i, clientSockets[i]);
            }
        }

        // Yield to prevent watchdog timeout
        taskYIELD();
    }

    // The task is the sole owner of cleanup(). stop() only joins on taskExited_,
    // and never touches taskHandle_ until this flag is observed set — so we must
    // not write taskHandle_ here (that would race the join).
    bridge->running_.store(false);
    bridge->cleanup();
    ESP_LOGI(TAG, "Bridge task exiting for bridge %d", bridge->bridgeId_);
    bridge->taskExited_.store(true);
    vTaskDelete(nullptr);
}

} // namespace tcp_cat_bridge
