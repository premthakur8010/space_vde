#include "space_docking/utils/Logger.hpp"
#include "space_docking/dynamics_engine/Types.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace space_docking::utils
{
	namespace
	{
		const char* getLevelString(LogLevel level) noexcept
		{
			switch (level)
			{
				case LogLevel::Trace: return "[TRACE]";
				case LogLevel::Debug: return "[DEBUG]";
				case LogLevel::Info:  return "[INFO ]";
				case LogLevel::Warn:  return "[WARN ]";
				case LogLevel::Error: return "[ERROR]";
			}
			return "[UNKN ]";
		}
	}

	Logger& Logger::getInstance() noexcept
	{
		static Logger instance;
		return instance;
	}

	Logger::~Logger()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_fileStream.is_open())
		{
			m_fileStream.flush();
			m_fileStream.close();
		}
	}

	void Logger::setLevel(LogLevel level) noexcept
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_level = level;
	}

	LogLevel Logger::getLevel() const noexcept
	{
		return m_level;
	}

	void Logger::setOutputFile(const std::string& filePath)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_fileStream.is_open())
		{
			m_fileStream.close();
		}
		m_fileStream.open(filePath, std::ios::out | std::ios::app);
	}

	void Logger::log(LogLevel level, std::string_view message)
	{
		if (static_cast<uint8_t>(level) < static_cast<uint8_t>(m_level))
		{
			return;
		}

		const auto now = std::chrono::system_clock::now();
		const auto inTimeT = std::chrono::system_clock::to_time_t(now);
		const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

		std::tm buf{};
#if defined(_WIN32)
		localtime_s(&buf, &inTimeT);
#else
		localtime_r(&inTimeT, &buf);
#endif

		std::ostringstream ss;
		ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S")
		   << '.' << std::setfill('0') << std::setw(3) << ms.count() << " "
		   << getLevelString(level) << " "
		   << message << "\n";

		const std::string formatted = ss.str();

		std::lock_guard<std::mutex> lock(m_mutex);
		std::cout << formatted;
		if (m_fileStream.is_open())
		{
			m_fileStream << formatted;
			m_fileStream.flush();
		}
	}

	void Logger::logWithContext(LogLevel level, std::string_view message,
	                            std::string_view file, int line,
	                            std::string_view func)
	{
		// 1. Check if message severity meets the configured minimum threshold
		if (static_cast<uint8_t>(level) < static_cast<uint8_t>(m_level))
		{
			return;
		}

		// 2. Compute current timestamp with millisecond precision
		const auto now = std::chrono::system_clock::now();
		const auto inTimeT = std::chrono::system_clock::to_time_t(now);
		const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

		std::tm buf{};
#if defined(_WIN32)
		localtime_s(&buf, &inTimeT);
#else
		localtime_r(&inTimeT, &buf);
#endif

		// 3. Strip full directory path to only show basename (e.g. "XmlConfigLoader.cpp")
		std::string_view filename = file;
		const auto lastSlash = filename.find_last_of("/\\");
		if (lastSlash != std::string_view::npos)
		{
			filename.remove_prefix(lastSlash + 1);
		}

		// 4. Format header: YYYY-MM-DD HH:MM:SS.mmm [LEVEL] (File:Line in func): message
		std::ostringstream ss;
		ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S")
		   << '.' << std::setfill('0') << std::setw(3) << ms.count() << " "
		   << getLevelString(level);

		if (!filename.empty() && line > 0)
		{
			ss << " (" << filename << ":" << line;
			if (!func.empty())
			{
				ss << " in " << func;
			}
			ss << "): ";
		}
		else
		{
			ss << " ";
		}

		ss << message << "\n";

		const std::string formatted = ss.str();

		// 5. Thread-safe write to two destinations:
		//    Destination A: stdout terminal / console (immediate developer visibility)
		//    Destination B: disk log file (if setOutputFile() was configured)
		std::lock_guard<std::mutex> lock(m_mutex);
		std::cout << formatted;
		if (m_fileStream.is_open())
		{
			m_fileStream << formatted;
			m_fileStream.flush(); // Flush immediately so logs persist even on abnormal exit or crash
		}
	}

	void Logger::logError(dynamics::ErrorCode code, std::string_view message,
	                      std::string_view file, int line,
	                      std::string_view func)
	{
		// Formats message as: "[ERROR_CODE_STRING] <custom message>" and routes to logWithContext
		std::ostringstream ss;
		ss << "[" << dynamics::errorCodeToString(code) << "] " << message;
		logWithContext(LogLevel::Error, ss.str(), file, line, func);
	}

	void Logger::trace(std::string_view message)
	{
		log(LogLevel::Trace, message);
	}

	void Logger::debug(std::string_view message)
	{
		log(LogLevel::Debug, message);
	}

	void Logger::info(std::string_view message)
	{
		log(LogLevel::Info, message);
	}

	void Logger::warn(std::string_view message)
	{
		log(LogLevel::Warn, message);
	}

	void Logger::error(std::string_view message)
	{
		log(LogLevel::Error, message);
	}
}
