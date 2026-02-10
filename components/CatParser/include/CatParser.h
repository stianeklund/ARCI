#pragma once

#include "RadioCommand.h"
#include "ParserUtils.h"
#include <string_view>
#include <optional>
#include <functional>

namespace radio {

/**
 * @brief Statistics for unified CAT parsing
 */
struct UnifiedCATStatistics {
    size_t totalMessagesParsed{0};
    size_t localCommandsParsed{0};
    size_t remoteCommandsParsed{0};
    size_t setCommands{0};
    size_t readCommands{0};
    size_t answerCommands{0};
    size_t parseErrors{0};
    size_t unknownCommands{0};

    void reset() {
        totalMessagesParsed = 0;
        localCommandsParsed = 0;
        remoteCommandsParsed = 0;
        setCommands = 0;
        readCommands = 0;
        answerCommands = 0;
        parseErrors = 0;
        unknownCommands = 0;
    }
};

/**
 * @brief Unified CAT parser that handles commands from both local (USB) and remote (radio) sources
 * 
 * This parser replaces the separate LocalCATHandler and RadioCATHandler with a single,
 * unified parser that can distinguish command types based on source context and content.
 */
class CatParser {
public:
    using CommandCallback = std::function<void(const RadioCommand&)>;
    using ErrorCallback = std::function<void(std::string_view)>;

    explicit CatParser();
    ~CatParser() = default;

    // Non-copyable, non-movable
    CatParser(const CatParser&) = delete;
    CatParser& operator=(const CatParser&) = delete;
    CatParser(CatParser&&) = delete;
    CatParser& operator=(CatParser&&) = delete;

    /**
     * @brief Parse a message containing one or more CAT commands
     * @param message The raw CAT message string
     * @param source Source of the message (Local from USB, Remote from radio)
     * @param callback Function to call for each parsed command
     */
    void parseMessage(std::string_view message, CommandSource source, CommandCallback callback);

    /**
     * @brief Set error callback for parse errors
     */
    void setErrorCallback(ErrorCallback callback) {
        errorCallback_ = std::move(callback);
    }

    /**
     * @brief Get current parsing statistics
     */
    const UnifiedCATStatistics& getStatistics() const { return stats_; }

    /**
     * @brief Reset parsing statistics
     */
    void resetStatistics() { stats_.reset(); }

private:
    /**
     * @brief Parse a single CAT command/response
     * @param frame Individual CAT frame (e.g., "FA00014150000;")
     * @param source Source of the frame
     * @return Parsed RadioCommand if successful, nullopt otherwise
     */
    std::optional<RadioCommand> parseFrame(std::string_view frame, CommandSource source);

    static CommandType determineSetCommandType(CommandSource source, std::string_view prefix, std::string_view params);
    static CommandType setOnlyCommandPrefixes(std::string_view prefix);

    /**
     * @brief Determine command type based on frame content and source
     * @param frame The CAT frame to analyze
     * @param source The source of the frame
     * @return Inferred CommandType
     */
    static CommandType determineCommandType(std::string_view frame, CommandSource source);

    /**
     * @brief Parse parameters from a CAT frame directly into RadioCommand
     * @param command The RadioCommand to populate with parameters (uses SSO inline storage)
     * @param frame The CAT frame
     * @param commandPrefix The command prefix (e.g., "FA")
     *
     * Uses RadioCommand::addParam() for zero-allocation parameter storage on most commands.
     */
    static void parseParameters(RadioCommand& command, std::string_view frame, std::string_view commandPrefix);

    /**
     * @brief Check if frame is a known error response
     * @param frame The frame to check
     * @return true if it's an error response (?;, E;, O;)
     */
    static bool isErrorResponse(std::string_view frame);

    /**
     * @brief Report a parse error
     */
    void reportError(std::string_view error) const;

    UnifiedCATStatistics stats_;
    ErrorCallback errorCallback_;

    static constexpr const char* TAG = "CatParser";
};

} // namespace radio