/**
 * @file XmlConfigLoader.hpp
 * @brief XML configuration parser for simulation initial parameters (masses, inertias, poses).
 *
 * Reads config/simulation_config.xml and populates the RunConfig structure without
 * requiring third-party XML dependencies (pure standard C++20).
 */

#pragma once

#include "space_docking/dynamics_engine/Types.hpp"
#include <string>

namespace space_docking::dynamics
{
	/**
	 * @brief Loads simulation run configurations from an XML configuration file.
	 */
	class XmlConfigLoader
	{
	public:
		/**
		 * @brief Parses an XML file into a RunConfig struct.
		 * @param filePath Path to the XML file.
		 * @param outConfig Populated configuration structure.
		 * @return ErrorCode::OK on success, error code on failure.
		 */
		static ErrorCode loadRunConfig(const std::string& filePath, RunConfig& outConfig);

		/**
		 * @brief Convenience loader returning populated RunConfig or default on error.
		 */
		static RunConfig loadOrDefault(const std::string& filePath);
	};
}
