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
	constexpr const char* kChaser          = "chaser";
	constexpr const char* kTarget          = "target";
	constexpr const char* kMassKg          = "masskg";
	constexpr const char* kInertia         = "inertia";
	constexpr const char* kIxx             = "ixx";
	constexpr const char* kIyy             = "iyy";
	constexpr const char* kIzz             = "izz";
	constexpr const char* kInitialPosition = "initialposition";
	constexpr const char* kInitialVelocity = "initialvelocity";
	constexpr const char* kX               = "x";
	constexpr const char* kY               = "y";
	constexpr const char* kZ               = "z";
	constexpr const char* kVx              = "vx";
	constexpr const char* kVy              = "vy";
	constexpr const char* kVz              = "vz";

	// Platform block
	constexpr const char* kPlatform        = "platform";
	constexpr const char* kFrameMapping    = "framemapping";

	// Logging block
	constexpr const char* kLogging         = "logging";
	constexpr const char* kEnabled         = "enabled";
	constexpr const char* kLogDirectory    = "logdirectory";
	constexpr const char* kLogFileName     = "logfilename";
	constexpr const char* kLogDecimation   = "logdecimation";
}

namespace space_docking::dynamics::xml_values
{
	// <framemapping> values
	constexpr const char* kFrameMappingDefault = "default";
	constexpr const char* kFrameMappingFlipped = "flipped";

	// <enabled> values
	constexpr const char* kTrue                = "true";
	constexpr const char* kFalse               = "false";
	constexpr const char* kOne                 = "1";
	constexpr const char* kZero                = "0";
}
