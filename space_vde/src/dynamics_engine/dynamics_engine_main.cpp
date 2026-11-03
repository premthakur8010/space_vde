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
	std::atomic<bool> g_running{true};

	void signalHandler(int /*signal*/)
	{
		g_running = false;
	}

	space_docking::dynamics::EulerAngles quatToEuler(const space_docking::dynamics::Quaternion& q)
	{
		space_docking::dynamics::EulerAngles euler;
		const double w = q.w, x = q.x, y = q.y, z = q.z;
		euler.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
		const double sinp = 2.0 * (w * y - z * x);
		euler.pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
		euler.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
		return euler;
	}
}

int main(int argc, char** argv)
{
#if defined(_WIN32)
	timeBeginPeriod(1);
#endif
	std::string configPath = "config/simulation_config.xml";
	std::string targetHost = space_docking::network::vde_net::kDefaultHost; // 127.0.0.1
	uint16_t statePort = space_docking::network::vde_net::kDefaultStatePort; // 5002
	uint16_t configPort = space_docking::network::vde_net::kDefaultConfigPort; // 5001

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

	space_docking::dynamics::RunConfig config{};
	// 1. Load XML configuration file and handle potential configuration errors
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

	// Initialize UDP sockets for UI Client communication (Approach A)
	space_docking::network::NetSystem::initialize();

	space_docking::network::UdpSocket stateSender;
	if (!stateSender.open())
	{
		SD_LOG_ERROR(space_docking::dynamics::ErrorCode::NETWORK_ERROR, "Failed to open stateSender UDP socket");
	}
	stateSender.setNonBlocking(true);

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

	std::signal(SIGINT, signalHandler);

	std::cout << "Starting 1 ms cyclic simulation loop with UDP streaming to "
	          << targetHost << ":" << statePort << " (125 Hz). Press Ctrl+C to terminate." << std::endl;

	uint64_t stepCount = 0;
	constexpr double dt = 0.001; // 1 ms physics period

	while (g_running)
	{
		const auto tickStart = std::chrono::steady_clock::now();

		// Check for incoming RunConfigPacket from UI client
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

		engine.step(dt);
		stepCount++;

		// Broadcast StatePacket to 120 Hz UI Client (~every 8 steps = 125 Hz)
		if (stepCount % 8 == 0)
		{
			space_docking::network::vde_net::StatePacket pkt{};
			pkt.magic = space_docking::network::vde_net::kProtocolMagic;
			pkt.version = space_docking::network::vde_net::kProtocolVersion;
			pkt.sequence = stepCount;
			pkt.timeS = stepCount * dt;

			const auto& rel = engine.getRelativeState();
			// Native space_vde scalar-first quaternion (w, x, y, z)
			pkt.relative.orientation = { rel.pose.orientation.w, rel.pose.orientation.x, rel.pose.orientation.y, rel.pose.orientation.z };
			pkt.relative.positionM = { rel.pose.position.x, rel.pose.position.y, rel.pose.position.z };
			pkt.relative.orientationRate = { rel.quaternionRate.w, rel.quaternionRate.x, rel.quaternionRate.y, rel.quaternionRate.z };
			pkt.relative.velocityMS = { rel.velocity.x, rel.velocity.y, rel.velocity.z };

			// Spacecraft dynamic telemetry
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

			// VDE Stewart Platform hardware representation:
			// In the VDE facility, the vertical axis (+Z) is the docking approach axis towards the overhead gantry.
			// Spacecraft Line-of-Sight approach (+X) maps to VDE vertical stroke (+Z, 0.0 to 0.550 m).
			// Spacecraft lateral misalignments (+Y, +Z) map to VDE horizontal platform displacement (+X, +Y, +/-0.100 m).
			const auto relEuler = rel.toEuler();
			const double initX = config.initialSeparation.position.x;
			double vdeZ = 0.0;
			double vdeVz = 0.0;
			if (std::abs(initX) > 1e-4)
			{
				const double progress = std::clamp((rel.pose.position.x - initX) / (0.0 - initX), 0.0, 1.0);
				vdeZ = progress * 0.550; // physical stroke: 0.0 to 550 mm
				vdeVz = (rel.velocity.x / (0.0 - initX)) * 0.550;
			}
			else
			{
				vdeZ = std::clamp(0.550 + rel.pose.position.x, 0.0, 0.550);
				vdeVz = rel.velocity.x;
			}

			const double vdeX = std::clamp(rel.pose.position.y, -0.100, 0.100);
			const double vdeY = std::clamp(rel.pose.position.z, -0.100, 0.100);
			const double vdeVx = rel.velocity.y;
			const double vdeVy = rel.velocity.z;

			// Angular attitude mapped to Stewart platform physical limits (Roll/Pitch +/-5 deg, Yaw +/-10 deg)
			const double vdeRoll  = std::clamp(relEuler.pitch, -0.087, 0.087);
			const double vdePitch = std::clamp(relEuler.yaw,   -0.087, 0.087);
			const double vdeYaw   = std::clamp(relEuler.roll,  -0.174, 0.174);

			const double vdeRollRate  = chaserState.angularRate.y - targetState.angularRate.y;
			const double vdePitchRate = chaserState.angularRate.z - targetState.angularRate.z;
			const double vdeYawRate   = chaserState.angularRate.x - targetState.angularRate.x;

			pkt.vde.positionM = { vdeX, vdeY, vdeZ };
			pkt.vde.linearRateMS = { vdeVx, vdeVy, vdeVz };
			pkt.vde.orientationRad = { vdeRoll, vdePitch, vdeYaw };
			pkt.vde.orientationRateRadS = { vdeRollRate, vdePitchRate, vdeYawRate };
			// ForceTorque: torque first, force second!
			pkt.vde.moving = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
			pkt.vde.overhead = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };

			pkt.ngcChaser = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
			pkt.ngcTarget = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };

			pkt.disturbancesOn = 1;
			std::snprintf(pkt.ngcMode, sizeof(pkt.ngcMode), "FINAL APPROACH (closing)");
			std::snprintf(pkt.ngcFailureCase, sizeof(pkt.ngcFailureCase), "None (nominal)");

			stateSender.sendTo(&pkt, sizeof(pkt), targetHost.c_str(), statePort);
		}

		// Print trajectory update every 500 ms (500 steps)
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

		// High-precision hybrid wait to maintain exact 1 ms (1000 Hz) cyclic rate without pinning CPU at 100%
		while (true)
		{
			const auto elapsed = std::chrono::steady_clock::now() - tickStart;
			if (elapsed >= std::chrono::microseconds(1000))
			{
				break;
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
	timeEndPeriod(1);
#endif
	return 0;
}

