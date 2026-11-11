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
	inline constexpr const char*   kDefaultHost       = "127.0.0.1";
	inline constexpr std::uint16_t kDefaultConfigPort = 5001; // Outbound: UI -> Engine
	inline constexpr std::uint16_t kDefaultStatePort  = 5002; // Inbound:  Engine -> UI
	inline constexpr std::uint32_t kProtocolMagic     = 0x56444531u; // ASCII "VDE1"
	inline constexpr std::uint32_t kProtocolVersion   = 1u;

	// Canonical type aliases directly mapped to Dynamics Engine single-source-of-truth types
	using Vec3 = space_docking::dynamics::Vector3;
	using Rpy = space_docking::dynamics::EulerAngles;
	using Mat3 = space_docking::dynamics::Matrix3;
	using ForceTorque = space_docking::dynamics::ForceTorque;
	using Quaternion = space_docking::dynamics::Quaternion;
	using RelativeState = space_docking::dynamics::RelativeState;
	using VdeState = space_docking::dynamics::VdeState;
	using SpacecraftState = space_docking::dynamics::SpacecraftState;

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
}
#pragma pack(pop)
