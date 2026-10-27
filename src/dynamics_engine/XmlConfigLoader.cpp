/**
 * @file XmlConfigLoader.cpp
 * @brief Implementation of XML configuration file parsing for the Dynamics Engine.
 */

#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"

#include <fstream>
#include <sstream>
#include <string>

namespace space_docking::dynamics
{
	namespace
	{
		// Extracts text between <tagName> and </tagName> inside a parent block
		std::string extractTagContent(const std::string& xml, const std::string& tagName)
		{
			const std::string openTag = "<" + tagName + ">";
			const std::string closeTag = "</" + tagName + ">";

			const size_t start = xml.find(openTag);
			if (start == std::string::npos)
			{
				return "";
			}

			const size_t contentStart = start + openTag.length();
			const size_t end = xml.find(closeTag, contentStart);
			if (end == std::string::npos)
			{
				return "";
			}

			return xml.substr(contentStart, end - contentStart);
		}

		double extractDouble(const std::string& xml, const std::string& tagName, double defaultValue = 0.0)
		{
			const std::string content = extractTagContent(xml, tagName);
			if (content.empty())
			{
				return defaultValue;
			}
			try
			{
				return std::stod(content);
			}
			catch (...)
			{
				return defaultValue;
			}
		}
	}

	ErrorCode XmlConfigLoader::loadRunConfig(const std::string& filePath, RunConfig& outConfig)
	{
		std::ifstream file(filePath);
		if (!file.is_open())
		{
			return ErrorCode::INVALID_ARGUMENT;
		}

		std::stringstream buffer;
		buffer << file.rdbuf();
		const std::string xml = buffer.str();

		// Parse Chaser block
		const std::string chaserBlock = extractTagContent(xml, "Chaser");
		if (!chaserBlock.empty())
		{
			outConfig.chaserMass = extractDouble(chaserBlock, "MassKg", 0.0);

			const std::string inertiaBlock = extractTagContent(chaserBlock, "Inertia");
			if (!inertiaBlock.empty())
			{
				outConfig.chaserInertia.m[0] = extractDouble(inertiaBlock, "Ixx", 0.0);
				outConfig.chaserInertia.m[4] = extractDouble(inertiaBlock, "Iyy", 0.0);
				outConfig.chaserInertia.m[8] = extractDouble(inertiaBlock, "Izz", 0.0);
			}

			const std::string posBlock = extractTagContent(chaserBlock, "InitialPosition");
			if (!posBlock.empty())
			{
				outConfig.initialSeparation.position.x = extractDouble(posBlock, "X", 0.0);
				outConfig.initialSeparation.position.y = extractDouble(posBlock, "Y", 0.0);
				outConfig.initialSeparation.position.z = extractDouble(posBlock, "Z", 0.0);
			}

			const std::string velBlock = extractTagContent(chaserBlock, "InitialVelocity");
			if (!velBlock.empty())
			{
				outConfig.chaserInitialVelocity.x = extractDouble(velBlock, "Vx", 0.0);
				outConfig.chaserInitialVelocity.y = extractDouble(velBlock, "Vy", 0.0);
				outConfig.chaserInitialVelocity.z = extractDouble(velBlock, "Vz", 0.0);
			}
		}

		// Parse Target block
		const std::string targetBlock = extractTagContent(xml, "Target");
		if (!targetBlock.empty())
		{
			outConfig.targetMass = extractDouble(targetBlock, "MassKg", 0.0);

			const std::string inertiaBlock = extractTagContent(targetBlock, "Inertia");
			if (!inertiaBlock.empty())
			{
				outConfig.targetInertia.m[0] = extractDouble(inertiaBlock, "Ixx", 0.0);
				outConfig.targetInertia.m[4] = extractDouble(inertiaBlock, "Iyy", 0.0);
				outConfig.targetInertia.m[8] = extractDouble(inertiaBlock, "Izz", 0.0);
			}

			const std::string velBlock = extractTagContent(targetBlock, "InitialVelocity");
			if (!velBlock.empty())
			{
				outConfig.targetInitialVelocity.x = extractDouble(velBlock, "Vx", 0.0);
				outConfig.targetInitialVelocity.y = extractDouble(velBlock, "Vy", 0.0);
				outConfig.targetInitialVelocity.z = extractDouble(velBlock, "Vz", 0.0);
			}
		}

		// Parse Platform mapping block
		const std::string platformBlock = extractTagContent(xml, "Platform");
		if (!platformBlock.empty())
		{
			const std::string mappingStr = extractTagContent(platformBlock, "FrameMapping");
			if (mappingStr == "FLIPPED")
			{
				outConfig.frameMapping = FrameMapping::FLIPPED;
			}
			else
			{
				outConfig.frameMapping = FrameMapping::DEFAULT;
			}
		}

		// Parse Logging configuration provision
		const std::string loggingBlock = extractTagContent(xml, "Logging");
		if (!loggingBlock.empty())
		{
			const std::string enabledStr = extractTagContent(loggingBlock, "Enabled");
			outConfig.loggingEnabled = (enabledStr == "true" || enabledStr == "1");

			const std::string logDir = extractTagContent(loggingBlock, "LogDirectory");
			if (!logDir.empty())
			{
				outConfig.logDirectory = logDir;
			}

			const std::string logFile = extractTagContent(loggingBlock, "LogFileName");
			if (!logFile.empty())
			{
				outConfig.logFileName = logFile;
			}

			const std::string decimationStr = extractTagContent(loggingBlock, "LogDecimation");
			if (!decimationStr.empty())
			{
				try
				{
					outConfig.logDecimation = static_cast<uint32_t>(std::stoul(decimationStr));
				}
				catch (...)
				{
					outConfig.logDecimation = 10;
				}
			}
		}

		return ErrorCode::OK;
	}

	RunConfig XmlConfigLoader::loadOrDefault(const std::string& filePath)
	{
		RunConfig config{};
		loadRunConfig(filePath, config);
		return config;
	}
}
