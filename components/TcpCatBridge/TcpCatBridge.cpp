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
    running_.store(false);

    // Wait for task to exit
    if (taskHandle_ != nullptr) {
        // Give task time to cleanup gracefully
        vTaskDelay(pdMS_TO_TICKS(100));

        // Force delete if still running
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }

    cleanup();
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

    // Bind to port
    struct sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(serverPort_);

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

    // Single client enforcement: close existing client before accepting new one
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (clients_[i].connected) {
            ESP_LOGI(TAG, "Closing existing client %d to accept new connection on bridge %d", i, bridgeId_);
            closeClient(i);
            break;  // Only one client can be connected at a time
        }
    }

    // Find free slot (should be available after closing existing client)
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (!clients_[i].connected) {
            clients_[i].socket = clientSock;
            clients_[i].connected = true;
            clients_[i].bytesRx = 0;
            clients_[i].bytesTx = 0;
            clients_[i].pendingLen = 0;

            // Set client socket non-blocking
            int flags = fcntl(clientSock, F_GETFL, 0);
            fcntl(clientSock, F_SETFL, flags | O_NONBLOCK);

            clientCount_.fetch_add(1);
            ESP_LOGI(TAG, "Client %d connected to bridge %d from %s:%d",
                     i, bridgeId_, inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
            return i;
        }
    }

    // Should not reach here due to single client enforcement
    ESP_LOGW(TAG, "No free slots (unexpected), rejecting connection");
    close(clientSock);
    return -1;
}

void TcpCatBridge::handleClientData(int clientIdx) {
    ClientState& client = clients_[clientIdx];

    ssize_t received = recv(client.socket, client.rxBuffer, BUFFER_SIZE, 0);

    if (received > 0) {
        client.bytesRx += static_cast<uint64_t>(received);
        bytesReceived_.fetch_add(static_cast<uint64_t>(received));

        ESP_LOGV(TAG, "Bridge %d received %d bytes from client %d", bridgeId_, (int)received, clientIdx);

        // Process input and invoke frame callback for complete CAT commands
        processClientInput(clientIdx, client.rxBuffer, static_cast<size_t>(received));
    } else if (received == 0) {
        // Client disconnected gracefully
        ESP_LOGI(TAG, "Client %d disconnected from bridge %d (RX=%llu, TX=%llu)",
                 clientIdx, bridgeId_, client.bytesRx, client.bytesTx);
        closeClient(clientIdx);
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

    ClientState& client = clients_[clientIdx];

    std::function<void(std::string_view)> frameCallback;
    {
        RtosLockGuard<RtosMutex> lock(callbackMutex_);
        frameCallback = incomingFrameCallback_;
    }

    for (size_t i = 0; i < length; ++i) {
        const unsigned char ch = data[i];

        // Strip all control characters except ';' (matches USB CDC sanitization)
        if (isControlChar(ch)) {
            continue;
        }

        // Handle buffer overflow: drop oldest data, keep newest (matches USB CDC behavior)
        if (client.pendingLen >= COMMAND_BUFFER_SIZE) {
            // Find the start of the most recent partial command (after last ';')
            size_t lastDelim = 0;
            for (size_t j = 0; j < client.pendingLen; ++j) {
                if (client.pendingBuffer[j] == ';') {
                    lastDelim = j + 1;
                }
            }

            if (lastDelim > 0 && lastDelim < client.pendingLen) {
                // Keep only the partial command after the last ';'
                const size_t keep = client.pendingLen - lastDelim;
                memmove(client.pendingBuffer.data(), client.pendingBuffer.data() + lastDelim, keep);
                client.pendingLen = keep;
                ESP_LOGW(TAG, "Client %d buffer overflow: dropped %zu bytes, kept %zu",
                         clientIdx, lastDelim, keep);
            } else {
                // No delimiter found or nothing after it - drop half the buffer
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
            if (frameCallback && client.pendingLen > 0) {
                std::string_view frame(client.pendingBuffer.data(), client.pendingLen);
                ESP_LOGV(TAG, "Bridge %d RX from client %d: %.*s", bridgeId_, clientIdx,
                         (int)frame.size(), frame.data());
                frameCallback(frame);
            }
            client.pendingLen = 0;
        }
    }
}

void TcpCatBridge::sendToActiveClient(std::string_view message) {
    if (message.empty()) {
        return;
    }

    // Send to the first connected client (single client enforcement ensures only one exists)
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (clients_[i].connected) {
            ESP_LOGV(TAG, "Bridge %d TX to client %d: %.*s", bridgeId_, i,
                     (int)message.size(), message.data());
            ssize_t sent = send(clients_[i].socket, message.data(), message.size(), 0);

            if (sent > 0) {
                clients_[i].bytesTx += static_cast<uint64_t>(sent);
                bytesSent_.fetch_add(static_cast<uint64_t>(sent));
            } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "Client %d send error: errno %d", i, errno);
                closeClient(i);
            }
            return;  // Single client only - done after first send attempt
        }
    }

    // No clients connected - silently drop message
    ESP_LOGV(TAG, "No clients connected on bridge %d, dropping message: %.*s",
             bridgeId_, static_cast<int>(message.size()), message.data());
}

void TcpCatBridge::closeClient(int clientIdx) {
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
    client.pendingLen = 0;
    clientCount_.fetch_sub(1);
}

void TcpCatBridge::closeAllClients() {
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        closeClient(i);
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
    struct timeval timeout{};

    while (bridge->running_.load()) {
        FD_ZERO(&readfds);

        // Add server socket to set
        int maxfd = bridge->serverSocket_;
        FD_SET(bridge->serverSocket_, &readfds);

        // Add client sockets to set
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (bridge->clients_[i].connected) {
                int sock = bridge->clients_[i].socket;
                FD_SET(sock, &readfds);
                if (sock > maxfd) {
                    maxfd = sock;
                }
            }
        }

        // Set timeout for select
        timeout.tv_sec = 0;
        timeout.tv_usec = SELECT_TIMEOUT_MS * 1000;

        // Wait for activity
        int activity = select(maxfd + 1, &readfds, nullptr, nullptr, &timeout);

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

        // Check for client data
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (bridge->clients_[i].connected &&
                FD_ISSET(bridge->clients_[i].socket, &readfds)) {
                bridge->handleClientData(i);
            }
        }

        // Yield to prevent watchdog timeout
        taskYIELD();
    }

    // Cleanup on exit
    bridge->running_.store(false);
    bridge->taskHandle_ = nullptr;
    bridge->cleanup();
    ESP_LOGI(TAG, "Bridge task exiting for bridge %d", bridge->bridgeId_);
    vTaskDelete(nullptr);
}

} // namespace tcp_cat_bridge
