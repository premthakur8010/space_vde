#pragma once

#include <cstdint>

/**
 * @brief Network wire protocol definitions and packed message structures.
 * Units: Positions in meters, velocities in m/s, time in seconds, period in us/ns.
 * Thread-safety: Struct definitions are immutable once transmitted.
 */
namespace space_docking::network
{
	constexpr uint16_t kDefaultSimStatePort = 9870;
	constexpr uint16_t kDefaultSimCommandPort = 9871;
	constexpr const char* kDefaultHost = "127.0.0.1";
	constexpr const char* kDefaultMulticastGroup = "239.255.0.1";

	constexpr uint32_t kSnapshotMsgMagic = 0x53504143; // 'SPAC'
	constexpr uint32_t kInputCommandMsgMagic = 0x434D4E44; // 'CMND'

	enum class SimCommandId : uint32_t
	{
		Scenario1 = 1,
		Scenario2 = 2,
		Reset = 3,
		TogglePause = 4
	};

#pragma pack(push, 1)

	/**
	 * @brief Wire-format telemetry snapshot packet broadcast by the dynamic engine.
	 */
	struct SnapshotMsg
	{
		uint32_t magic{kSnapshotMsgMagic};
		uint64_t tick{0};
		double simTimeSec{0.0};

		// Target Spacecraft (Cube 1)
		double targetPosM[3]{0.0, 0.0, 0.0};
		double targetQuat[4]{1.0, 0.0, 0.0, 0.0}; // w, x, y, z
		double targetVelMps[3]{0.0, 0.0, 0.0};

		// Chaser Spacecraft (Cube 2)
		double chaserPosM[3]{0.0, 0.0, 0.0};
		double chaserQuat[4]{1.0, 0.0, 0.0, 0.0}; // w, x, y, z
		double chaserVelMps[3]{0.0, 0.0, 0.0};

		// Simulation state indicators
		uint8_t contactOccurred{0};
		uint8_t paused{0};
		uint8_t scenarioId{1};
		uint8_t secondsSpecified{0};

		// Precision timing telemetry
		uint32_t physicsPeriodUs{1000};
		uint32_t avgPeriodNs{0};
		uint32_t maxPeriodNs{0};
		uint32_t maxWorkNs{0};
		uint32_t maxLatencyNs{0};
		uint64_t latencyViolations{0};
		uint64_t lateTicks{0};
		double jitterToleranceUs{100.0};
		double headlessSeconds{0.0};
	};

	/**
	 * @brief Wire-format interactive command packet sent from UI to engine.
	 */
	struct InputCommandMsg
	{
		uint32_t magic{kInputCommandMsgMagic};
		SimCommandId command{SimCommandId::Reset};
	};

#pragma pack(pop)
}
