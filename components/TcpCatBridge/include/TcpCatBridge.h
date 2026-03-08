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
     * @brief Handle data from TCP client -> CAT handler (via callback)
     * @param clientIdx Client index in clients_ array
     */
    void handleClientData(int clientIdx);

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

    // Client connection state
    struct ClientState {
        int socket = -1;
        bool connected = false;
        uint8_t rxBuffer[BUFFER_SIZE]{};
        uint64_t bytesRx = 0;
        uint64_t bytesTx = 0;
        std::array<char, COMMAND_BUFFER_SIZE> pendingBuffer{};
        size_t pendingLen = 0;
    };

    // State
    uint8_t bridgeId_;      // Bridge identifier (0 or 1) for CommandSource::Tcp0/Tcp1
    uint16_t serverPort_;   // TCP port for this bridge
    int serverSocket_ = -1;
    ClientState clients_[MAX_CLIENTS]{};
    TaskHandle_t taskHandle_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<uint8_t> clientCount_{0};
    std::atomic<uint64_t> bytesReceived_{0};
    std::atomic<uint64_t> bytesSent_{0};
    std::function<void(std::string_view)> incomingFrameCallback_;
    mutable RtosMutex callbackMutex_;
    mutable RtosMutex clientsMutex_;  // Protects clients_[] access across tasks
};

} // namespace tcp_cat_bridge
