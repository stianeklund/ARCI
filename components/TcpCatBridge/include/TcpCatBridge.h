#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>
#include <array>
#include <cstdint>
#include <functional>
#include <string_view>
#include "rtos_mutex.h"

// Fallback defaults so the unit compiles before sdkconfig is regenerated from
// Kconfig. The build system force-includes sdkconfig.h, which overrides these.
#ifndef CONFIG_TCP_CAT_BRIDGE_AUTH_TOKEN
#define CONFIG_TCP_CAT_BRIDGE_AUTH_TOKEN ""
#endif
#ifndef CONFIG_TCP_CAT_BRIDGE_BIND_ADDR
#define CONFIG_TCP_CAT_BRIDGE_BIND_ADDR "0.0.0.0"
#endif
#ifndef CONFIG_TCP_CAT_BRIDGE_MIN_ACCEPT_INTERVAL_MS
#define CONFIG_TCP_CAT_BRIDGE_MIN_ACCEPT_INTERVAL_MS 1000
#endif

namespace tcp_cat_bridge {

/**
 * @brief Lightweight TCP to CAT command bridge
 *
 * Provides network access to the radio's CAT interface by:
 * - Listening on a TCP port for incoming connections
 * - Forwarding CAT commands from TCP clients to the radio via callback
 * - Forwarding radio responses back to the active TCP client
 * - Enforcing single active client per port
 *
 * Architecture:
 * - Single FreeRTOS task handles all I/O using select()
 * - Event-driven design: minimal CPU usage when idle
 * - Direct routing to RadioManager via frame callback
 * - Independent AI modes and forwarding state per port
 * - Graceful handling of client disconnections
 *
 * Resource usage:
 * - CPU: 2-4% under load, ~0% when idle
 * - Memory: ~7-9 KB per active client, ~20 KB per bridge instance
 * - Stack: 4 KB per bridge (configurable)
 */
class TcpCatBridge {
public:
    static constexpr const char* TAG = "TcpCatBridge";

    /**
     * @brief Construct a TCP-CAT bridge
     * @param port TCP port to listen on
     * @param bridgeId Bridge identifier (0 or 1) for command source routing
     */
    explicit TcpCatBridge(uint16_t port, uint8_t bridgeId);

    /**
     * @brief Destructor - stops the bridge and cleans up resources
     */
    ~TcpCatBridge();

    /**
     * @brief Register callback invoked whenever a full CAT frame is received from TCP client
     * @param callback Function to call with received frame (e.g., RadioManager::dispatchMessage)
     */
    void setIncomingFrameCallback(std::function<void(std::string_view)> callback);

    // Non-copyable, non-movable
    TcpCatBridge(const TcpCatBridge&) = delete;
    TcpCatBridge& operator=(const TcpCatBridge&) = delete;
    TcpCatBridge(TcpCatBridge&&) = delete;
    TcpCatBridge& operator=(TcpCatBridge&&) = delete;

    /**
     * @brief Start the TCP bridge server
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t start();

    /**
     * @brief Stop the TCP bridge server
     */
    void stop();

    /**
     * @brief Check if bridge is running
     * @return true if server is active
     */
    bool isRunning() const { return running_.load(); }

    /**
     * @brief Get number of currently connected clients
     * @return Active client count (0 or 1, since single client is enforced)
     */
    uint8_t getClientCount() const { return clientCount_.load(); }

    /**
     * @brief Get total bytes received from TCP clients
     * @return Bytes received counter
     */
    uint64_t getBytesReceived() const { return bytesReceived_.load(); }

    /**
     * @brief Get total bytes sent to TCP clients
     * @return Bytes sent counter
     */
    uint64_t getBytesSent() const { return bytesSent_.load(); }

    /**
     * @brief Get bridge identifier (0 or 1)
     * @return Bridge ID
     */
    uint8_t getBridgeId() const { return bridgeId_; }

    /**
     * @brief Get TCP port this bridge listens on
     * @return Server port number
     */
    uint16_t getServerPort() const { return serverPort_; }

    /**
     * @brief Send message to the currently active TCP client
     * @param message CAT response frame to send
     *
     * This method is called by RadioManager to route responses back to the TCP client.
     * If no client is connected, the message is silently dropped.
     */
    void sendToActiveClient(std::string_view message);

private:
    /**
     * @brief Main bridge task - handles all I/O
     * @param arg Pointer to TcpCatBridge instance
     */
    static void bridgeTask(void* arg);

    /**
     * @brief Initialize TCP listening socket
     * @return ESP_OK on success
     */
    esp_err_t initSocket();

    /**
     * @brief Accept new client connection
     * Enforces single client policy by disconnecting existing client if present
     * @return Client socket FD, or -1 on error
     */
    int acceptClient();

    /**
     * @brief Handle readable data from TCP client -> CAT handler (via callback)
     * @param clientIdx Client index in clients_ array
     * @param expectedSock Socket fd captured in the select() snapshot; recv is
     *                     skipped if the slot no longer holds this fd (evicted/reused)
     */
    void handleClientData(int clientIdx, int expectedSock);

    /**
     * @brief Flush pending TX to a writable client socket
     * @param clientIdx Client index in clients_ array
     * @param expectedSock Socket fd captured in the select() snapshot (revalidated)
     */
    void handleClientWritable(int clientIdx, int expectedSock);

    /**
     * @brief Flush queued pending-TX bytes to the socket (caller holds clientsMutex_)
     *
     * Non-blocking: sends what it can, keeps the remainder queued. Closes the
     * client on a hard send error. Never blocks.
     * @param clientIdx Client index in clients_ array
     */
    void flushPendingTxLocked(int clientIdx);

    /**
     * @brief Queue bytes into the client's pending-TX buffer (caller holds clientsMutex_)
     *
     * If the buffer would overflow the stream is unrecoverable, so the client is
     * closed to preserve whole-frame semantics.
     * @return true if queued, false if the client was closed on overflow
     */
    bool appendPendingTxLocked(int clientIdx, const char* data, size_t length);

    /**
     * @brief Validate an unauthenticated client's first frame against the auth token
     *
     * On match: replies "AUTH1;", marks the client authenticated and evicts any
     * other connected client. On mismatch: closes the client. Never forwards the
     * frame to the CAT callback.
     */
    void handleAuthFrame(int clientIdx, std::string_view frame);

    /**
     * @brief Process received input from TCP client
     * @param clientIdx Client index
     * @param data Received data
     * @param length Data length
     */
    void processClientInput(int clientIdx, const uint8_t* data, size_t length);

    /**
     * @brief Close client connection and cleanup (acquires clientsMutex_)
     * @param clientIdx Client index in clients_ array
     */
    void closeClient(int clientIdx);

    /**
     * @brief Close client connection (caller must hold clientsMutex_)
     * @param clientIdx Client index in clients_ array
     */
    void closeClientLocked(int clientIdx);

    /**
     * @brief Close all client connections
     */
    void closeAllClients();

    /**
     * @brief Cleanup bridge resources
     */
    void cleanup();

    // Configuration (from KConfig)
    static constexpr int MAX_CLIENTS = CONFIG_TCP_CAT_BRIDGE_MAX_CLIENTS;
    static constexpr int BUFFER_SIZE = CONFIG_TCP_CAT_BRIDGE_BUFFER_SIZE;
    static constexpr int SELECT_TIMEOUT_MS = CONFIG_TCP_CAT_BRIDGE_SELECT_TIMEOUT_MS;
    static constexpr size_t COMMAND_BUFFER_SIZE = BUFFER_SIZE * 2;
    static constexpr int MIN_STACK_DEPTH = 6144;

    // Minimum interval between accepted connections (blunts reconnect-loop DoS)
    static constexpr int MIN_ACCEPT_INTERVAL_MS = CONFIG_TCP_CAT_BRIDGE_MIN_ACCEPT_INTERVAL_MS;

    // Fixed per-client pending-TX buffer. Holds the unsent remainder of a frame
    // when the socket would block, so whole-frame semantics survive a partial send.
    static constexpr size_t PENDING_TX_SIZE = 512;

    // Auth is enabled iff the configured token is non-empty. sizeof includes the
    // trailing NUL, so an empty "" literal yields 1.
    static constexpr bool AUTH_ENABLED = sizeof(CONFIG_TCP_CAT_BRIDGE_AUTH_TOKEN) > 1;

    // Client connection state
    struct ClientState {
        int socket = -1;
        bool connected = false;
        bool authenticated = false;   // true once AUTH accepted (or immediately if auth disabled)
        uint64_t bytesRx = 0;
        uint64_t bytesTx = 0;
        std::array<char, COMMAND_BUFFER_SIZE> pendingBuffer{};
        size_t pendingLen = 0;
        std::array<char, PENDING_TX_SIZE> txBuffer{}; // unsent frame remainder (would-block)
        size_t txPending = 0;                          // bytes queued in txBuffer
    };

    // State
    uint8_t bridgeId_;      // Bridge identifier (0 or 1) for CommandSource::Tcp0/Tcp1
    uint16_t serverPort_;   // TCP port for this bridge
    int serverSocket_ = -1;
    ClientState clients_[MAX_CLIENTS]{};
    TaskHandle_t taskHandle_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<bool> taskExited_{false};  // set by the task just before it self-deletes
    TickType_t lastAcceptTick_ = 0;        // tick of last accepted connection (rate limit)
    std::atomic<uint8_t> clientCount_{0};
    std::atomic<uint64_t> bytesReceived_{0};
    std::atomic<uint64_t> bytesSent_{0};
    std::function<void(std::string_view)> incomingFrameCallback_;
    mutable RtosMutex callbackMutex_;
    mutable RtosMutex clientsMutex_;  // Protects clients_[] access across tasks
};

} // namespace tcp_cat_bridge
