#pragma once

#include "space_docking/dynamics_engine/Types.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace space_docking::dynamics
{
	/**
	 * @brief Error logger recording error conditions and diagnostics.
	 */
	class EngineLogger
	{
	public:
		static EngineLogger& getInstance() noexcept
		{
			static EngineLogger instance;
			return instance;
		}

		void log(LogLevel level, ErrorCode code, const std::string& message = "")
		{
			const char* levelStr = (level == LogLevel::FATAL) ? "[FATAL]" :
			                       (level == LogLevel::ERROR) ? "[ERROR]" : "[WARN]";
			std::cerr << levelStr << " ErrorCode: " << static_cast<int32_t>(code);
			if (!message.empty())
			{
				std::cerr << " - " << message;
			}
			std::cerr << std::endl;
		}

	private:
		EngineLogger() = default;
	};

	/**
	 * @brief CSV telemetry and simulation logger.
	 */
	class CsvLogger
	{
	public:
		explicit CsvLogger(const std::string& filename)
			: m_file(filename, std::ios::out | std::ios::trunc)
		{
		}

		~CsvLogger()
		{
			if (m_file.is_open())
			{
				m_file.close();
			}
		}

		ErrorCode writeRow(const std::vector<double>& values)
		{
			if (!m_file.is_open())
			{
				return ErrorCode::STEP_FAILED;
			}

			for (size_t i = 0; i < values.size(); ++i)
			{
				m_file << values[i];
				if (i + 1 < values.size())
				{
					m_file << ",";
				}
			}
			m_file << "\n";
			return ErrorCode::OK;
		}

	private:
		std::ofstream m_file;
	};
}
