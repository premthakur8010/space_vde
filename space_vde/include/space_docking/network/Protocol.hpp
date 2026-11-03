#pragma once

#include <cstdint>
#include <array>

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

	// =========================================================================
	// VDE Docking Simulator Wire Protocol (for ui-space-docking-system-refactored)
	// =========================================================================
#pragma pack(push, 1)
	namespace vde_net
	{
		inline constexpr const char*   kDefaultHost       = "127.0.0.1";
		inline constexpr std::uint16_t kDefaultConfigPort = 5001; // Outbound: UI -> Engine
		inline constexpr std::uint16_t kDefaultStatePort  = 5002; // Inbound:  Engine -> UI
		inline constexpr std::uint32_t kProtocolMagic     = 0x56444531u; // ASCII "VDE1"
		inline constexpr std::uint32_t kProtocolVersion   = 1u;

		struct Vec3 { double x = 0.0, y = 0.0, z = 0.0; };
		struct Rpy  { double roll = 0.0, pitch = 0.0, yaw = 0.0; };
		struct Mat3 {
			std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
			constexpr Mat3() noexcept = default;
			constexpr Mat3(const std::array<double, 9>& elements) noexcept : m(elements) {}
		};
		struct ForceTorque {
			Vec3 torque{};
			Vec3 force{};
		};
		struct Quaternion { double w = 1.0, x = 0.0, y = 0.0, z = 0.0; }; // scalar-first (w, x, y, z)

		struct RelativeState {
			Quaternion orientation;
			Vec3       positionM;
			Quaternion orientationRate;
			Vec3       velocityMS;
		};

		struct VdeState {
			Vec3        positionM, linearRateMS;
			Rpy         orientationRad, orientationRateRadS;
			ForceTorque moving;   // torque first, force second
			ForceTorque overhead; // torque first, force second
		};

		struct SpacecraftState {
			double massKg = 0.0;
			Vec3   cgM;
			Mat3   inertiaKgM2;
			Vec3   linearRateMS;
			Rpy    orientationRateRadS;
			Vec3   eciPositionM, eciVelocityMS;
			Rpy    eciOrientationRad, eciOrientationRateRadS;
		};

		struct RunConfigPacket {
			std::uint32_t magic          = kProtocolMagic;
			std::uint32_t version        = kProtocolVersion;
			double        chaserMassKg   = 2150.0;
			double        targetMassKg   = 7800.0;
			Mat3          chaserInertia;
			Mat3          targetInertia;
			double        orbitRadiusM   = 6778.0e3;
			double        inclinationRad = 0.0;
			double        alongTrackSepM = -12.0;
			std::int32_t  frameMapping   = 0;
			std::uint8_t  disturbancesOn = 1;
			char          ngcMode[64]    = {};
		};

		struct StatePacket {
			std::uint32_t   magic          = kProtocolMagic;
			std::uint32_t   version        = kProtocolVersion;
			std::uint64_t   sequence       = 0;
			double          timeS          = 0.0;
			RelativeState   relative;
			VdeState        vde;
			SpacecraftState chaser;
			SpacecraftState target;
			ForceTorque     ngcChaser; // torque first, force second
			ForceTorque     ngcTarget; // torque first, force second
			std::uint8_t    disturbancesOn = 1;
			char            ngcMode[64]        = {};
			char            ngcFailureCase[64] = {};
		};

		static_assert(sizeof(RunConfigPacket) == 261, "RunConfigPacket must be exactly 261 bytes");
		static_assert(sizeof(StatePacket) == 1049, "StatePacket must be exactly 1049 bytes");
	}
#pragma pack(pop)
}
