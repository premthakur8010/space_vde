/**
 * @file EngineLogger.hpp
 * @brief Aggregate header providing diagnostic error logging and CSV telemetry capabilities.
 *
 * PURPOSE:
 * --------
 * Unifies the lightweight code-level EngineLogger (for FATAL, ERROR, WARN console diagnostics)
 * and the high-throughput asynchronous CsvLogger (for telemetry & 3D visualization replay).
 *
 * Implements the Logger and CsvLogger component interfaces defined in the PES University
 * Software Design Document (SDD Section 4 and Figure 3).
 */

#pragma once

#include "space_docking/dynamics_engine/CsvLogger.hpp"
#include "space_docking/dynamics_engine/Types.hpp"

#include <iostream>
#include <string>

namespace space_docking::dynamics
{
	/**
	 * @class EngineLogger
	 * @brief Error logger recording FATAL, ERROR, and WARNING diagnostic events.
	 *
	 * Provides a thread-safe singleton for logging system-level status codes and messages
	 * directly to stderr or console diagnostics.
	 */
	class EngineLogger
	{
	public:
		/**
		 * @brief Returns the thread-safe singleton EngineLogger instance.
		 */
		static EngineLogger& getInstance() noexcept
		{
			static EngineLogger instance;
			return instance;
		}

		/**
		 * @brief Log an error condition with severity level and ErrorCode.
		 * @param level Log severity level (FATAL, ERROR, WARNING).
		 * @param code Standard ErrorCode enum value.
		 * @param message Optional contextual description.
		 */
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
}
