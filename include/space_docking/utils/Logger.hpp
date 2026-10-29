#pragma once

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>

/**
 * @brief Centralized project logger with severity filtering and file persistence.
 * Preconditions: Thread-safe for multi-threaded logging.
 */
namespace space_docking::dynamics
{
	enum class ErrorCode : int32_t;
}

namespace space_docking::utils
{
	enum class LogLevel : uint8_t
	{
		Trace = 0,
		Debug,
		Info,
		Warn,
		Error
	};

	class Logger
	{
	public:
		/// Returns the thread-safe singleton Logger instance
		static Logger& getInstance() noexcept;

		/// Sets the minimum severity threshold for filtering messages
		void setLevel(LogLevel level) noexcept;
		LogLevel getLevel() const noexcept;

		/**
		 * @brief Directs logger to write messages to a file in append mode.
		 * All future log messages will be mirrored to this file as well as the console.
		 */
		void setOutputFile(const std::string& filePath);

		/// Basic log message without source file/line context
		void log(LogLevel level, std::string_view message);

		/**
		 * @brief Logs a formatted message with timestamp, severity, filename, line, and function.
		 * Formatted output is written to std::cout and mirrored to m_fileStream (if configured).
		 */
		void logWithContext(LogLevel level, std::string_view message,
		                    std::string_view file = "", int line = 0,
		                    std::string_view func = "");

		/**
		 * @brief Specialized error logger that prefixes the message with the ErrorCode enum name.
		 */
		void logError(dynamics::ErrorCode code, std::string_view message,
		              std::string_view file = "", int line = 0,
		              std::string_view func = "");

		void trace(std::string_view message);
		void debug(std::string_view message);
		void info(std::string_view message);
		void warn(std::string_view message);
		void error(std::string_view message);

	private:
		Logger() = default;
		~Logger();

		Logger(const Logger&) = delete;
		Logger& operator=(const Logger&) = delete;

		LogLevel m_level{LogLevel::Info};
		std::ofstream m_fileStream;
		std::mutex m_mutex;
	};
}
