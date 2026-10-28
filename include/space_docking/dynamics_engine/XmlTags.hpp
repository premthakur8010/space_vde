/**
 * @file XmlTags.hpp
 * @brief Tag names and accepted values for simulation_config.xml.
 *
 * Keep these in sync with config/simulation_config.xml. Tag names are case-sensitive.
 */

#pragma once

namespace space_docking::dynamics::xml_tags
{
	// Spacecraft blocks
	constexpr const char* kChaser          = "Chaser";
	constexpr const char* kTarget          = "Target";
	constexpr const char* kMassKg          = "MassKg";
	constexpr const char* kInertia         = "Inertia";
	constexpr const char* kIxx             = "Ixx";
	constexpr const char* kIyy             = "Iyy";
	constexpr const char* kIzz             = "Izz";
	constexpr const char* kInitialPosition = "InitialPosition";
	constexpr const char* kInitialVelocity = "InitialVelocity";
	constexpr const char* kX               = "X";
	constexpr const char* kY               = "Y";
	constexpr const char* kZ               = "Z";
	constexpr const char* kVx              = "Vx";
	constexpr const char* kVy              = "Vy";
	constexpr const char* kVz              = "Vz";

	// Platform block
	constexpr const char* kPlatform        = "Platform";
	constexpr const char* kFrameMapping    = "FrameMapping";

	// Logging block
	constexpr const char* kLogging         = "Logging";
	constexpr const char* kEnabled         = "Enabled";
	constexpr const char* kLogDirectory    = "LogDirectory";
	constexpr const char* kLogFileName     = "LogFileName";
	constexpr const char* kLogDecimation   = "LogDecimation";
}

namespace space_docking::dynamics::xml_values
{
	// <FrameMapping> values
	constexpr const char* kFrameMappingFlipped = "FLIPPED";

	// <Enabled> values
	constexpr const char* kTrue                = "true";
	constexpr const char* kOne                 = "1";
}
