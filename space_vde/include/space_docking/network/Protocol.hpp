/**
 * @file Protocol.hpp
 * @brief Canonical network wire protocol definitions for VDE Docking Simulator.
 *
 * Provides binary-packed packet structures for UDP communication between
 * the Dynamics Engine (physics server) and UI/visualization clients.
 *
 * Wire format guarantees:
 * - Little-endian byte order
 * - Fixed 1-byte structural packing (#pragma pack(push, 1))
 * - Single source of truth: derives from space_docking::dynamics::Types.hpp
 */

#pragma once

#include "../dynamics_engine/Types.hpp"
#include <cstdint>

#pragma pack(push, 1)
namespace space_docking::network::vde_net
{
	// Fallback UDP endpoint defaults used when <network> is omitted in simulation_config.xml
	inline constexpr const char*   kDefaultHost            = "127.0.0.1";
	inline constexpr std::uint16_t kDefaultConfigPort      = 5001; // Inbound:  UI -> Engine
	inline constexpr std::uint16_t kDefaultStatePort       = 5002; // Outbound: Engine -> UI
	inline constexpr std::uint16_t kDefaultHwTelemetryPort = 5005; // Inbound:  VDE Hardware/Emulator -> Engine
	inline constexpr std::uint16_t kDefaultHwCommandPort   = 5006; // Outbound: Engine -> VDE Hardware/Emulator
	inline constexpr std::uint32_t kProtocolMagic          = 0x56444531u; // ASCII "VDE1"
	inline constexpr std::uint32_t kProtocolVersion        = 1u;

	// Canonical type aliases directly mapped to Dynamics Engine single-source-of-truth types
	using Vec3 = space_docking::dynamics::Vector3;
	using Rpy = space_docking::dynamics::EulerAngles;
	using Mat3 = space_docking::dynamics::Matrix3;
	using Pose = space_docking::dynamics::Pose;
	using ForceTorque = space_docking::dynamics::ForceTorque;
	using Quaternion = space_docking::dynamics::Quaternion;
	using RelativeState = space_docking::dynamics::RelativeState;
	using VdeState = space_docking::dynamics::VdeState;
	using SpacecraftState = space_docking::dynamics::SpacecraftState;

	enum class HardwareCommandMode : std::uint8_t
	{
		TRACK_POSE = 0,
		STOP_HOLD  = 1,
		RESET_HOME = 2
	};

	/**
	 * @brief Fixed-layout binary datagram sent from UI client to Dynamics Engine to configure or update a run.
	 *
	 * WIRE SPECIFICATION:
	 * - Size: Exactly 261 bytes (enforced by #pragma pack(push, 1) and static_assert).
	 * - Direction: UI Client (outbound) -> Dynamics Engine (configured configPort, default 5001).
	 * - Update Semantics: Transmitted initially on simulator startup, and re-transmitted
	 *   dynamically whenever the operator changes masses, inertias, or platform mountings in the GUI.
	 */
	struct RunConfigPacket
	{
		std::uint32_t magic          = kProtocolMagic;   // Magic header identifier: 0x56444531 ("VDE1")
		std::uint32_t version        = kProtocolVersion; // Protocol revision: 1
		double        chaserMassKg   = 2150.0;           // Chaser wet mass [kg]
		double        targetMassKg   = 7800.0;           // Target wet mass [kg]
		Mat3          chaserInertia{};                   // Chaser 3x3 inertia tensor (9 doubles = 72 bytes)
		Mat3          targetInertia{};                   // Target 3x3 inertia tensor (9 doubles = 72 bytes)
		double        orbitRadiusM   = 6778.0e3;         // Orbital semi-major axis radius [m] (e.g. LEO ~400 km)
		double        inclinationRad = 0.0;              // Orbital inclination [rad]
		double        alongTrackSepM = -12.0;            // Initial along-track separation distance [m]
		std::int32_t  frameMapping   = 0;                // 0 = DEFAULT (Chaser on GSP), 1 = FLIPPED (Target on GSP)
		std::uint8_t  disturbancesOn = 1;                // 1 = Orbital disturbances active, 0 = pure kinematics
		char          ngcMode[64]    = {};               // Active Navigation, Guidance & Control mode string
	};

	/**
	 * @brief Fixed-layout binary datagram streamed from Dynamics Engine to UI client (125 Hz).
	 *
	 * WIRE SPECIFICATION:
	 * - Size: Exactly 1049 bytes (enforced by #pragma pack(push, 1) and static_assert).
	 * - Direction: Dynamics Engine (outbound) -> UI Client (configured statePort, default 5002).
	 * - Rate: 125 Hz (downsampled from 1000 Hz physics loop: stepCount % 8 == 0).
	 * - Telemetry Contents:
	 *   1. Sequence counter and simulation epoch timestamp.
	 *   2. Relative docking state (relative position, orientation quaternion, closing velocity).
	 *   3. Hardware VDE Stewart platform kinematics and load-cell readings.
	 *   4. Chaser & Target individual 6-DOF body states and ECI orbital vectors.
	 *   5. NGC Guidance thruster wrench commands.
	 */
	struct StatePacket
	{
		std::uint32_t   magic          = kProtocolMagic;   // Magic header: 0x56444531 ("VDE1")
		std::uint32_t   version        = kProtocolVersion; // Protocol revision: 1
		std::uint64_t   sequence       = 0;                // Monotonic packet sequence counter
		double          timeS          = 0.0;              // Elapsed simulation time [s]
		RelativeState   relative{};                        // 14-element relative docking state
		VdeState        vde{};                             // Physical testbed Stewart platform representation
		SpacecraftState chaser{};                          // Chaser mass, rates, and ECI coordinates
		SpacecraftState target{};                          // Target mass, rates, and ECI coordinates
		ForceTorque     ngcChaser{};                       // Chaser guidance thruster command (torque, then force)
		ForceTorque     ngcTarget{};                       // Target guidance thruster command (torque, then force)
		std::uint8_t    disturbancesOn = 1;                // 1 if disturbances are simulated, 0 otherwise
		char            ngcMode[64]        = {};           // Guidance mode text (e.g. "FINAL APPROACH (closing)")
		char            ngcFailureCase[64] = {};           // Active fault injection description
	};

	static_assert(sizeof(RunConfigPacket) == 261, "RunConfigPacket must be exactly 261 bytes");
	static_assert(sizeof(StatePacket) == 1049, "StatePacket must be exactly 1049 bytes");

	/**
	 * @brief Master HIL command datagram sent from Dynamics Engine to VDE Hardware / Emulator (500 Hz).
	 *
	 * WIRE SPECIFICATION (Master Contract):
	 * - Size: Exactly 105 bytes (#pragma pack(push, 1), little-endian SI units).
	 * - Direction: Dynamics Engine (outbound) -> VDE Hardware / Emulator (configured hardwareCommandPort, default 5006).
	 * - Coordinate Frame: Canonical space_vde Simulation Frame (docs/conventions.md):
	 *   +X = docking approach (-0.550 m initial separation -> 0.0 m contact -> >0.0 m penetration),
	 *   +Y = up [m], +Z = lateral right [m], orientation = scalar-first unit quaternion q(w,x,y,z).
	 */
	struct HardwareCommandPacket
	{
		std::uint32_t magic       = kProtocolMagic;   // Magic header: 0x56444531 ("VDE1")
		std::uint32_t version     = kProtocolVersion; // Protocol revision: 1
		std::uint64_t sequence    = 0;                // Monotonic command packet counter
		double        timeS       = 0.0;              // Elapsed simulation time [s]
		std::uint8_t  commandMode = 0;                // 0 = TRACK_POSE, 1 = STOP_HOLD, 2 = RESET_HOME
		Pose          relativePose{};                 // Commanded relative docking pose (56 bytes)
		Vec3          relativeVelocity{};             // Commanded relative linear velocity [m/s] (24 bytes)
	};

	/**
	 * @brief Master HIL telemetry datagram streamed from VDE Hardware / Emulator to Dynamics Engine.
	 *
	 * WIRE SPECIFICATION (Master Contract):
	 * - Size: Exactly 225 bytes (#pragma pack(push, 1), little-endian SI units).
	 * - Direction: VDE Hardware / Emulator (outbound) -> Dynamics Engine (configured hardwareTelemetryPort, default 5005).
	 * - Coordinate Frame: Canonical space_vde Simulation Frame (+X docking, +Y up, +Z lateral right; q(w,x,y,z)).
	 * - ForceTorque Order: Matches Types.hpp::ForceTorque strictly (torque [N*m] FIRST, force [N] SECOND).
	 */
	struct HardwareTelemetryPacket
	{
		std::uint32_t magic               = kProtocolMagic;   // Magic header: 0x56444531 ("VDE1")
		std::uint32_t version             = kProtocolVersion; // Protocol revision: 1
		std::uint64_t sequence            = 0;                // Monotonic hardware/emulator tick counter
		double        timeS               = 0.0;              // Hardware elapsed time [s]
		Pose          measuredRelativePose{};                 // FK-solved / measured relative pose in Simulation Frame (56 bytes)
		double        legLengths[6]{};                        // 6 Stewart platform actuator leg lengths [m] (48 bytes)
		ForceTorque   gspForceTorque{};                       // Moving/GSP load-cell wrench: torque [N*m], then force [N] (48 bytes)
		ForceTorque   fspForceTorque{};                       // Overhead/FSP load-cell wrench: torque [N*m], then force [N] (48 bytes)
		std::uint8_t  contactActive       = 0;                // 1 = docking contact active, 0 = free motion
	};

	static_assert(sizeof(HardwareCommandPacket) == 105, "HardwareCommandPacket must be exactly 105 bytes");
	static_assert(sizeof(HardwareTelemetryPacket) == 225, "HardwareTelemetryPacket must be exactly 225 bytes");
}
#pragma pack(pop)
