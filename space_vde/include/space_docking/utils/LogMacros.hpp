/**
 * @file LogMacros.hpp
 * @brief Ergonomic logging macros for the Space Docking Simulation.
 *
 * Provides streamable logging macros that automatically capture file, line, and function name
 * without verbose singleton invocations.
 */

#pragma once

#include "space_docking/dynamics_engine/Types.hpp"
#include "space_docking/utils/Logger.hpp"

#include <sstream>

// =============================================================================
// Streamable Logging Macros (captures source context: filename, line, function)
// =============================================================================

/**
 * @brief Logs an INFO message with automatic source context.
 * Usage: SD_LOG_INFO("Engine initialized with dt=" << dt << " s");
 */
#define SD_LOG_INFO(msg) \
	do { \
		std::ostringstream _sd_oss; \
		_sd_oss << msg; \
		::space_docking::utils::Logger::getInstance().logWithContext( \
			::space_docking::utils::LogLevel::Info, _sd_oss.str(), __FILE__, __LINE__, __func__); \
	} while (0)

/**
 * @brief Logs a WARNING message with automatic source context.
 * Usage: SD_LOG_WARN("Timestep jitter exceeded threshold: " << jitter << " ms");
 */
#define SD_LOG_WARN(msg) \
	do { \
		std::ostringstream _sd_oss; \
		_sd_oss << msg; \
		::space_docking::utils::Logger::getInstance().logWithContext( \
			::space_docking::utils::LogLevel::Warn, _sd_oss.str(), __FILE__, __LINE__, __func__); \
	} while (0)

/**
 * @brief Logs a DEBUG message with automatic source context (filtered in release unless enabled).
 * Usage: SD_LOG_DEBUG("Chaser state vector norm: " << stateNorm);
 */
#define SD_LOG_DEBUG(msg) \
	do { \
		std::ostringstream _sd_oss; \
		_sd_oss << msg; \
		::space_docking::utils::Logger::getInstance().logWithContext( \
			::space_docking::utils::LogLevel::Debug, _sd_oss.str(), __FILE__, __LINE__, __func__); \
	} while (0)

/**
 * @brief Logs a TRACE message for granular hot-path troubleshooting.
 * Usage: SD_LOG_TRACE("Entering step() integration cycle " << cycleCount);
 */
#define SD_LOG_TRACE(msg) \
	do { \
		std::ostringstream _sd_oss; \
		_sd_oss << msg; \
		::space_docking::utils::Logger::getInstance().logWithContext( \
			::space_docking::utils::LogLevel::Trace, _sd_oss.str(), __FILE__, __LINE__, __func__); \
	} while (0)

/**
 * @brief Logs an ERROR message coupled with an explicit typed ErrorCode enum.
 * Formats as: [ERROR] (File:Line in func): [ERROR_CODE_STRING] <your message>
 * Usage: SD_LOG_ERROR(ErrorCode::INVALID_MASS, "Chaser mass cannot be negative: " << mass);
 */
#define SD_LOG_ERROR(code, msg) \
	do { \
		std::ostringstream _sd_oss; \
		_sd_oss << msg; \
		::space_docking::utils::Logger::getInstance().logError( \
			(code), _sd_oss.str(), __FILE__, __LINE__, __func__); \
	} while (0)
