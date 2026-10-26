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
		static Logger& getInstance() noexcept;

		void setLevel(LogLevel level) noexcept;
		LogLevel getLevel() const noexcept;

		void setOutputFile(const std::string& filePath);

		void log(LogLevel level, std::string_view message);

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
