/**
 * @file XmlConfigLoader.cpp
 * @brief Implementation of XML configuration file parsing for the Dynamics Engine.
 */

#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"
#include "space_docking/dynamics_engine/XmlTags.hpp"
#include "space_docking/utils/LogMacros.hpp"
#include "space_docking/utils/MathUtils.hpp"

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
		// 1. Verify file accessibility on filesystem
		std::ifstream file(filePath);
		if (!file.is_open())
		{
			SD_LOG_ERROR(ErrorCode::CONFIG_FILE_NOT_FOUND, "Failed to open XML configuration file: " << filePath);
			return ErrorCode::CONFIG_FILE_NOT_FOUND;
		}

		// 2. Read whole file into buffer and verify non-empty content
		std::stringstream buffer;
		buffer << file.rdbuf();
		const std::string xml = buffer.str();

		if (xml.empty())
		{
			SD_LOG_ERROR(ErrorCode::CONFIG_PARSE_ERROR, "XML configuration file is empty: " << filePath);
			return ErrorCode::CONFIG_PARSE_ERROR;
		}

		namespace tags = xml_tags;
		namespace values = xml_values;

		// 3. Parse and validate Chaser block
		const std::string chaserBlock = extractTagContent(xml, tags::kChaser);
		if (chaserBlock.empty())
		{
			SD_LOG_ERROR(ErrorCode::CONFIG_MISSING_TAG, "Missing required <" << tags::kChaser << "> block in: " << filePath);
			return ErrorCode::CONFIG_MISSING_TAG;
		}
		{
			outConfig.chaserMass = extractDouble(chaserBlock, tags::kMassKg, 0.0);

			const std::string inertiaBlock = extractTagContent(chaserBlock, tags::kInertia);
			if (!inertiaBlock.empty())
			{
				outConfig.chaserInertia.m[0] = extractDouble(inertiaBlock, tags::kIxx, 0.0);
				outConfig.chaserInertia.m[4] = extractDouble(inertiaBlock, tags::kIyy, 0.0);
				outConfig.chaserInertia.m[8] = extractDouble(inertiaBlock, tags::kIzz, 0.0);
			}

			const std::string posBlock = extractTagContent(chaserBlock, tags::kInitialPosition);
			if (!posBlock.empty())
			{
				// XML positions are in millimetres (target reference frame); engine state is SI meters.
				outConfig.initialSeparation.position.x = extractDouble(posBlock, tags::kX, 0.0) * space_docking::utils::kMmToM;
				outConfig.initialSeparation.position.y = extractDouble(posBlock, tags::kY, 0.0) * space_docking::utils::kMmToM;
				outConfig.initialSeparation.position.z = extractDouble(posBlock, tags::kZ, 0.0) * space_docking::utils::kMmToM;
			}

			const std::string velBlock = extractTagContent(chaserBlock, tags::kInitialVelocity);
			if (!velBlock.empty())
			{
				outConfig.chaserInitialVelocity.x = extractDouble(velBlock, tags::kVx, 0.0);
				outConfig.chaserInitialVelocity.y = extractDouble(velBlock, tags::kVy, 0.0);
				outConfig.chaserInitialVelocity.z = extractDouble(velBlock, tags::kVz, 0.0);
			}

			// Validate physical mass parameter (must be positive to prevent division by zero in dynamics)
			if (outConfig.chaserMass <= 0.0)
			{
				SD_LOG_ERROR(ErrorCode::INVALID_MASS, "Chaser mass must be > 0.0 kg in " << filePath << ", parsed: " << outConfig.chaserMass);
				return ErrorCode::INVALID_MASS;
			}
		}

		// 4. Parse and validate Target block
		const std::string targetBlock = extractTagContent(xml, tags::kTarget);
		if (targetBlock.empty())
		{
			SD_LOG_ERROR(ErrorCode::CONFIG_MISSING_TAG, "Missing required <" << tags::kTarget << "> block in: " << filePath);
			return ErrorCode::CONFIG_MISSING_TAG;
		}
		{
			outConfig.targetMass = extractDouble(targetBlock, tags::kMassKg, 0.0);
			if (outConfig.targetMass <= 0.0)
			{
				SD_LOG_ERROR(ErrorCode::INVALID_MASS, "Target mass must be > 0.0 kg in " << filePath << ", parsed: " << outConfig.targetMass);
				return ErrorCode::INVALID_MASS;
			}

			const std::string inertiaBlock = extractTagContent(targetBlock, tags::kInertia);
			if (!inertiaBlock.empty())
			{
				outConfig.targetInertia.m[0] = extractDouble(inertiaBlock, tags::kIxx, 0.0);
				outConfig.targetInertia.m[4] = extractDouble(inertiaBlock, tags::kIyy, 0.0);
				outConfig.targetInertia.m[8] = extractDouble(inertiaBlock, tags::kIzz, 0.0);
			}

			const std::string velBlock = extractTagContent(targetBlock, tags::kInitialVelocity);
			if (!velBlock.empty())
			{
				outConfig.targetInitialVelocity.x = extractDouble(velBlock, tags::kVx, 0.0);
				outConfig.targetInitialVelocity.y = extractDouble(velBlock, tags::kVy, 0.0);
				outConfig.targetInitialVelocity.z = extractDouble(velBlock, tags::kVz, 0.0);
			}
		}

		// Parse Platform mapping block
		const std::string platformBlock = extractTagContent(xml, tags::kPlatform);
		if (!platformBlock.empty())
		{
			const std::string mappingStr = extractTagContent(platformBlock, tags::kFrameMapping);
			if (mappingStr == values::kFrameMappingFlipped)
			{
				outConfig.frameMapping = FrameMapping::FLIPPED;
			}
			else
			{
				outConfig.frameMapping = FrameMapping::DEFAULT;
			}
		}

		// Parse Logging configuration provision
		const std::string loggingBlock = extractTagContent(xml, tags::kLogging);
		if (!loggingBlock.empty())
		{
			const std::string enabledStr = extractTagContent(loggingBlock, tags::kEnabled);
			outConfig.loggingEnabled = (enabledStr == values::kTrue || enabledStr == values::kOne);

			const std::string logDir = extractTagContent(loggingBlock, tags::kLogDirectory);
			if (!logDir.empty())
			{
				outConfig.logDirectory = logDir;
			}

			const std::string logFile = extractTagContent(loggingBlock, tags::kLogFileName);
			if (!logFile.empty())
			{
				outConfig.logFileName = logFile;
			}

			const std::string decimationStr = extractTagContent(loggingBlock, tags::kLogDecimation);
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

		SD_LOG_INFO("Successfully loaded simulation configuration from " << filePath
		            << " (Chaser: " << outConfig.chaserMass << " kg, Target: " << outConfig.targetMass << " kg)");

		return ErrorCode::OK;
	}

	RunConfig XmlConfigLoader::loadOrDefault(const std::string& filePath)
	{
		RunConfig config{};
		loadRunConfig(filePath, config);
		return config;
	}
}
