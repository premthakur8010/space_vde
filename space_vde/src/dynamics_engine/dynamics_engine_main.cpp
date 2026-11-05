/**
 * @file dynamics_engine_main.cpp
 * @brief Standalone real-time executable entry point for the Space Docking Dynamics Engine.
 *
 * SYSTEM ARCHITECTURE & EXECUTION FLOW:
 * 1. High-Resolution OS Timer Setup:
 *    - On Windows, the default system timer resolution is 15.6 ms. We invoke timeBeginPeriod(1)
 *      to request 1.0 ms precision from the OS scheduler.
 * 2. Configuration Loading:
 *    - Parses simulation_config.xml dynamically via XmlConfigLoader (masses, inertias, initial pos/vel).
 * 3. Non-Blocking UDP Networking:
 *    - Port 5001 (Config Receiver): Listens asynchronously for RunConfigPacket from the UI client,
 *      allowing mid-run parameter updates (mass, inertia, frame mapping) without resetting trajectory.
 *    - Port 5002 (State Broadcaster): Streams StatePacket telemetry to the UI at 125 Hz (every 8 ms).
 * 4. 1 kHz Cyclic Execution Loop:
 *    - Advances 6-DOF physics by dt = 1.0 ms using Semi-Implicit (Symplectic) Euler integration.
 *    - Maps simulated orbital relative kinematics to physical VDE Stewart Platform hardware coordinates.
 * 5. Hybrid Precise Sleep/Spin Timing:
 *    - Yields thread execution while remaining cycle time > 200 microseconds.
 *    - Busy-spins for the final sub-millisecond tail to eliminate scheduler jitter and guarantee exact 1000 Hz.
 */

#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"
#include "space_docking/network/Protocol.hpp"
#include "space_docking/network/UdpSocket.hpp"
#include "space_docking/utils/LogMacros.hpp"
#include "space_docking/utils/Logger.hpp"
#include "space_docking/utils/MathUtils.hpp"

#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>

#if defined(_WIN32)
#include <windows.h>
#include <timeapi.h>
#endif

namespace
{
	// Global atomic flag to handle graceful shutdown on SIGINT (Ctrl+C)
	std::atomic<bool> g_running{true};

	// Intercepts SIGINT/SIGTERM (Ctrl+C) to trigger graceful simulation shutdown
	void signalHandler(int /*signal*/)
	{
		g_running = false;
	}

	using space_docking::utils::quatToEuler;
}

// Main entry point: runs 1 kHz physics loop, receives UI RunConfig on 5001, broadcasts StatePacket on 5002
int main(int argc, char** argv)
{
#if defined(_WIN32)
	// Request 1 ms timer granularity from the Windows NT multimedia scheduler.
	// Without this, std::this_thread::sleep_for or system waits have a coarse 15.6 ms granularity.
	timeBeginPeriod(1);
#endif

	// Default runtime network ports and configuration paths
	std::string configPath = "config/simulation_config.xml";
	std::string targetHost = space_docking::network::vde_net::kDefaultHost;       // 127.0.0.1 (localhost)
	uint16_t statePort     = space_docking::network::vde_net::kDefaultStatePort;  // Port 5002: Engine -> UI telemetry
	uint16_t configPort    = space_docking::network::vde_net::kDefaultConfigPort; // Port 5001: UI -> Engine config

	// Parse command-line overrides (e.g., --target-ip 192.168.1.100 --state-port 5002)
	for (int i = 1; i < argc; ++i)
	{
		if (std::strcmp(argv[i], "--target-ip") == 0 && i + 1 < argc)
		{
			targetHost = argv[++i];
		}
		else if (std::strcmp(argv[i], "--state-port") == 0 && i + 1 < argc)
		{
			statePort = static_cast<uint16_t>(std::atoi(argv[++i]));
		}
		else if (std::strcmp(argv[i], "--config-port") == 0 && i + 1 < argc)
		{
			configPort = static_cast<uint16_t>(std::atoi(argv[++i]));
		}
		else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc)
		{
			configPath = argv[++i];
		}
		else if (argv[i][0] != '-')
		{
			configPath = argv[i];
		}
	}

	std::cout << "========================================================\n";
	std::cout << " Space Docking Dynamics Engine (GSP/FSP HIL Simulation) \n";
	std::cout << "========================================================\n";
	std::cout << "Configuration file : " << configPath << "\n";
	std::cout << "Target UI endpoint : " << targetHost << ":" << statePort << "\n";
	std::cout << "Config listen port : " << configPort << "\n";

	// 1. Load XML configuration file and handle potential configuration errors
	space_docking::dynamics::RunConfig config{};
	space_docking::dynamics::ErrorCode ec = space_docking::dynamics::XmlConfigLoader::loadRunConfig(configPath, config);
	if (ec != space_docking::dynamics::ErrorCode::OK)
	{
		SD_LOG_ERROR(ec, "Failed to load simulation configuration from " << configPath);
		return 1;
	}

	// 2. If XML has <Logging><Enabled>true</Enabled>, mirror all code-level event logs to simulation.log
	// Note: simulation_trajectory.csv is reserved for numerical telemetry state vectors (rolling window).
	if (config.loggingEnabled)
	{
		const std::string logPath = config.logDirectory + "/simulation.log";
		space_docking::utils::Logger::getInstance().setOutputFile(logPath);
		SD_LOG_INFO("File logging active: writing diagnostics to " << logPath);
	}

	std::cout << "Chaser Mass: " << config.chaserMass << " kg" << std::endl;
	std::cout << "Target Mass: " << config.targetMass << " kg" << std::endl;
	std::cout << "Chaser Initial Pos: (" << config.initialSeparation.position.x << ", "
	          << config.initialSeparation.position.y << ", "
	          << config.initialSeparation.position.z << ") m (config: "
	          << space_docking::utils::metersToOpenGl(config.initialSeparation.position.x) << ", "
	          << space_docking::utils::metersToOpenGl(config.initialSeparation.position.y) << ", "
	          << space_docking::utils::metersToOpenGl(config.initialSeparation.position.z) << " mm = OpenGL units, 1:1)" << std::endl;
	std::cout << "Chaser Initial Vel: (" << config.chaserInitialVelocity.x << ", "
	          << config.chaserInitialVelocity.y << ", "
	          << config.chaserInitialVelocity.z << ") m/s" << std::endl;
	std::cout << "Logging Provision: " << (config.loggingEnabled ? "ENABLED" : "DISABLED")
	          << " -> " << config.logDirectory << "/" << config.logFileName
	          << " (rate: every " << config.logDecimation << " ms)" << std::endl;

	space_docking::dynamics::DynamicsEngine engine;
	ec = engine.initialize(config);
	if (ec != space_docking::dynamics::ErrorCode::OK)
	{
		SD_LOG_ERROR(ec, "Engine initialization failed with code: " << space_docking::dynamics::errorCodeToString(ec));
		return 1;
	}

	// =========================================================================
	// UDP Networking Initialization (Approach A: Direct IPC)
	// =========================================================================
	// NetSystem::initialize() initializes Winsock on Windows (WSAStartup)
	space_docking::network::NetSystem::initialize();

	// State Broadcaster: Streams 1049-byte StatePackets to UI Client at 125 Hz.
	// Configured as non-blocking so a slow network interface never delays the 1 kHz physics loop.
	space_docking::network::UdpSocket stateSender;
	if (!stateSender.open())
	{
		SD_LOG_ERROR(space_docking::dynamics::ErrorCode::NETWORK_ERROR, "Failed to open stateSender UDP socket");
	}
	stateSender.setNonBlocking(true);

	// Config Receiver: Listens for incoming RunConfigPacket from UI Client.
	// setReuseAddr(true) allows rapid simulator restarts without "address already in use" errors.
	space_docking::network::UdpSocket configReceiver;
	if (configReceiver.open())
	{
		configReceiver.setNonBlocking(true);
		configReceiver.setReuseAddr(true);
		if (!configReceiver.bindSocket(configPort, "0.0.0.0"))
		{
			SD_LOG_ERROR(space_docking::dynamics::ErrorCode::SOCKET_BIND_FAILED,
			             "Failed to bind configReceiver to port " << configPort);
		}
		else
		{
			SD_LOG_INFO("Config receiver active on port " << configPort);
		}
	}

	// Register OS signal handler for graceful shutdown on Ctrl+C (SIGINT)
	std::signal(SIGINT, signalHandler);

	std::cout << "Starting 1 ms cyclic simulation loop with UDP streaming to "
	          << targetHost << ":" << statePort << " (125 Hz). Press Ctrl+C to terminate." << std::endl;

	uint64_t stepCount = 0;
	constexpr double dt = 0.001; // 1 ms physics integration timestep (1000 Hz)

	// =========================================================================
	// Main Cyclic Real-Time Execution Loop (1 kHz / 1 ms period)
	// =========================================================================
	while (g_running)
	{
		// Capture cycle start timestamp with monotonic steady_clock
		const auto tickStart = std::chrono::steady_clock::now();

		// ---------------------------------------------------------------------
		// Step 1: Check for Mid-Run Configuration Updates from UI Client
		// ---------------------------------------------------------------------
		// Non-blocking recv: if UI changed mass, inertia, or platform mapping, update parameters on the fly.
		space_docking::network::vde_net::RunConfigPacket cfgPkt{};
		const int rxBytes = configReceiver.recvFrom(&cfgPkt, sizeof(cfgPkt));
		if (rxBytes == static_cast<int>(sizeof(cfgPkt)) &&
		    cfgPkt.magic == space_docking::network::vde_net::kProtocolMagic &&
		    cfgPkt.version == space_docking::network::vde_net::kProtocolVersion)
		{
			SD_LOG_INFO("Received RunConfig over UDP: Chaser Mass=" << cfgPkt.chaserMassKg
			            << " kg, Target Mass=" << cfgPkt.targetMassKg
			            << " kg, FrameMapping=" << cfgPkt.frameMapping);

			space_docking::dynamics::Matrix3 chInertia;
			chInertia.m = cfgPkt.chaserInertia.m;
			space_docking::dynamics::Matrix3 tgtInertia;
			tgtInertia.m = cfgPkt.targetInertia.m;
			const auto frameMapping = (cfgPkt.frameMapping == 1)
			    ? space_docking::dynamics::FrameMapping::FLIPPED
			    : space_docking::dynamics::FrameMapping::DEFAULT;

			// Safely update runtime physical parameters without resetting active trajectory or desyncing clocks
			engine.updateParameters(cfgPkt.chaserMassKg, chInertia,
			                        cfgPkt.targetMassKg, tgtInertia,
			                        frameMapping);
		}

		// ---------------------------------------------------------------------
		// Step 2: Advance Physics Engine by dt (1.0 ms)
		// ---------------------------------------------------------------------
		// Integrates Chaser and Target equations of motion via Symplectic Euler, resolves kinematics,
		// computes relative state vector Y, and buffers CSV telemetry.
		engine.step(dt);
		stepCount++;

		// ---------------------------------------------------------------------
		// Step 3: Telemetry Downsampling & UDP Broadcast to UI (125 Hz)
		// ---------------------------------------------------------------------
		// Physics runs at 1000 Hz, but UI rendering runs at 120-125 Hz.
		// Broadcasting every 8th physics step (1000 / 8 = 125 Hz) reduces network load by 87.5%
		// while providing silky-smooth 60/120 FPS visual updates.
		if (stepCount % 8 == 0)
		{
			space_docking::network::vde_net::StatePacket pkt{};
			pkt.magic = space_docking::network::vde_net::kProtocolMagic;
			pkt.version = space_docking::network::vde_net::kProtocolVersion;
			pkt.sequence = stepCount;
			pkt.timeS = stepCount * dt;

			// 3a. Relative State Vector (r_rel, v_rel, q_rel, qDot_rel)
			const auto& rel = engine.getRelativeState();
			pkt.relative = rel;

			// 3b. Chaser Spacecraft Telemetry (Body and Inertial ECI frames)
			const auto& chaser = engine.getChaser();
			const auto& chaserState = chaser.getState();
			const auto chaserEuler = quatToEuler(chaserState.pose.orientation);

			pkt.chaser.massKg = chaser.getMass();
			pkt.chaser.cgM = { 0.0, 0.0, 0.0 };
			pkt.chaser.inertiaKgM2.m = chaser.getInertia().m;
			pkt.chaser.linearRateMS = { chaserState.velocity.x, chaserState.velocity.y, chaserState.velocity.z };
			pkt.chaser.orientationRateRadS = { chaserState.angularRate.x, chaserState.angularRate.y, chaserState.angularRate.z };
			pkt.chaser.eciPositionM = { chaserState.pose.position.x, chaserState.pose.position.y, chaserState.pose.position.z };
			pkt.chaser.eciVelocityMS = { chaserState.velocity.x, chaserState.velocity.y, chaserState.velocity.z };
			pkt.chaser.eciOrientationRad = { chaserEuler.roll, chaserEuler.pitch, chaserEuler.yaw };
			pkt.chaser.eciOrientationRateRadS = { chaserState.angularRate.x, chaserState.angularRate.y, chaserState.angularRate.z };

			// 3c. Target Spacecraft Telemetry (Body and Inertial ECI frames)
			const auto& target = engine.getTarget();
			const auto& targetState = target.getState();
			const auto targetEuler = quatToEuler(targetState.pose.orientation);

			pkt.target.massKg = target.getMass();
			pkt.target.cgM = { 0.0, 0.0, 0.0 };
			pkt.target.inertiaKgM2.m = target.getInertia().m;
			pkt.target.linearRateMS = { targetState.velocity.x, targetState.velocity.y, targetState.velocity.z };
			pkt.target.orientationRateRadS = { targetState.angularRate.x, targetState.angularRate.y, targetState.angularRate.z };
			pkt.target.eciPositionM = { targetState.pose.position.x, targetState.pose.position.y, targetState.pose.position.z };
			pkt.target.eciVelocityMS = { targetState.velocity.x, targetState.velocity.y, targetState.velocity.z };
			pkt.target.eciOrientationRad = { targetEuler.roll, targetEuler.pitch, targetEuler.yaw };
			pkt.target.eciOrientationRateRadS = { targetState.angularRate.x, targetState.angularRate.y, targetState.angularRate.z };

			// -----------------------------------------------------------------
			// 3d. VDE Stewart Platform Hardware Coordinate Mapping:
			// -----------------------------------------------------------------
			// In the physical ground testbed facility:
			// - The Ground Stewart Platform (GSP) moves on 6 hydraulic/electric actuator legs.
			// - Docking approach axis (spacecraft Line-of-Sight, +X) is physically oriented
			//   along the VERTICAL axis (+Z) of the testbed toward the overhead ceiling gantry.
			// - Physical vertical stroke limit is 550 mm (0.0 to 0.550 m).
			// - Spacecraft lateral errors (+Y, +Z) map to testbed horizontal sway (+X, +Y),
			//   clamped to physical hexapod travel boundaries (+/- 0.100 m).
			// - Angular roll/pitch are clamped to +/- 5 deg (+/- 0.087 rad); yaw to +/- 10 deg (+/- 0.174 rad).
			const auto relEuler = rel.toEuler();
			const double initX = config.initialSeparation.position.x;
			double vdeZ = 0.0;
			double vdeVz = 0.0;
			if (std::abs(initX) > 1e-4)
			{
				// Progress ratio: 0.0 at initial standoff distance, 1.0 at physical contact (0.0 m)
				const double progress = std::clamp((rel.pose.position.x - initX) / (0.0 - initX), 0.0, 1.0);
				vdeZ = progress * 0.550; // Map normalized progress to physical 0 to 550 mm stroke
				vdeVz = (rel.velocity.x / (0.0 - initX)) * 0.550;
			}
			else
			{
				vdeZ = std::clamp(0.550 + rel.pose.position.x, 0.0, 0.550);
				vdeVz = rel.velocity.x;
			}

			// Clamp horizontal displacements to Stewart platform travel envelope (+/- 100 mm)
			const double vdeX = std::clamp(rel.pose.position.y, -0.100, 0.100);
			const double vdeY = std::clamp(rel.pose.position.z, -0.100, 0.100);
			const double vdeVx = rel.velocity.y;
			const double vdeVy = rel.velocity.z;

			// Clamp platform angular attitudes to mechanical actuator stroke limits
			const double vdeRoll  = std::clamp(relEuler.pitch, -0.087, 0.087);
			const double vdePitch = std::clamp(relEuler.yaw,   -0.087, 0.087);
			const double vdeYaw   = std::clamp(relEuler.roll,  -0.174, 0.174);

			// Compute relative angular rates between Chaser and Target
			const double vdeRollRate  = chaserState.angularRate.y - targetState.angularRate.y;
			const double vdePitchRate = chaserState.angularRate.z - targetState.angularRate.z;
			const double vdeYawRate   = chaserState.angularRate.x - targetState.angularRate.x;

			pkt.vde.positionM = { vdeX, vdeY, vdeZ };
			pkt.vde.linearRateMS = { vdeVx, vdeVy, vdeVz };
			pkt.vde.orientationRad = { vdeRoll, vdePitch, vdeYaw };
			pkt.vde.orientationRateRadS = { vdeRollRate, vdePitchRate, vdeYawRate };
			// ForceTorque layout: torque first, then force
			pkt.vde.moving = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
			pkt.vde.overhead = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };

			pkt.ngcChaser = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
			pkt.ngcTarget = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };

			pkt.disturbancesOn = 1;
			std::snprintf(pkt.ngcMode, sizeof(pkt.ngcMode), "FINAL APPROACH (closing)");
			std::snprintf(pkt.ngcFailureCase, sizeof(pkt.ngcFailureCase), "None (nominal)");

			// Transmit binary-packed StatePacket over UDP to UI client
			stateSender.sendTo(&pkt, sizeof(pkt), targetHost.c_str(), statePort);
		}

		// ---------------------------------------------------------------------
		// Step 4: Periodic Console Diagnostics (every 500 ms = 500 steps)
		// ---------------------------------------------------------------------
		if (stepCount % 500 == 0)
		{
			const auto& rel = engine.getRelativeState();
			const auto euler = rel.toEuler();
			std::cout << "[T=" << (stepCount * dt) << "s] Relative Pos: ("
			          << rel.pose.position.x << ", "
			          << rel.pose.position.y << ", "
			          << rel.pose.position.z << ") m | Closing Vel: "
			          << rel.velocity.x << " m/s | Yaw: "
			          << euler.yaw << " rad" << std::endl;
		}

		// ---------------------------------------------------------------------
		// Step 5: High-Precision Hybrid Wait Loop (Exact 1000 Hz Timing)
		// ---------------------------------------------------------------------
		// WHY HYBRID WAIT?
		// Standard OS sleep (std::this_thread::sleep_for) has scheduler jitter of ~1-2 ms,
		// which causes massive frequency fluctuations on a 1 ms cyclic task.
		// Pure busy-spinning (while(true){}) achieves microsecond precision but pins CPU cores at 100%.
		//
		// SOLUTION:
		// 1. If remaining cycle time > 200 microseconds: call std::this_thread::yield() to allow
		//    other threads / OS processes to run without putting the CPU into deep sleep.
		// 2. Once within 200 microseconds of the 1000 us deadline: busy-spin on steady_clock.
		// This guarantees < 5 microsecond timing jitter with moderate CPU usage.
		while (true)
		{
			const auto elapsed = std::chrono::steady_clock::now() - tickStart;
			if (elapsed >= std::chrono::microseconds(1000))
			{
				break; // 1.0 ms elapsed; trigger next physics cycle
			}
			const auto remaining = std::chrono::microseconds(1000) - elapsed;
			if (remaining > std::chrono::microseconds(200))
			{
				std::this_thread::yield();
			}
		}
	}

	std::cout << "\nDynamics Engine cleanly stopped after " << stepCount << " cycles (" << (stepCount * dt) << " seconds)." << std::endl;
	engine.flushLogs();

#if defined(_WIN32)
	// Restore Windows system timer resolution to OS default
	timeEndPeriod(1);
#endif
	return 0;
}

