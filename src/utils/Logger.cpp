#include "space_docking/utils/Logger.hpp"

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
