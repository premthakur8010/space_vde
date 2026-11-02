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
	if (argc > 1)
	{
		configPath = argv[1];
	}

	std::cout << "========================================================\n";
	std::cout << " Space Docking Dynamics Engine (GSP/FSP HIL Simulation) \n";
	std::cout << "========================================================\n";
	std::cout << "Loading configuration: " << configPath << "\n";

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
		if (!configReceiver.bindSocket(space_docking::network::vde_net::kDefaultConfigPort, "0.0.0.0"))
		{
			SD_LOG_ERROR(space_docking::dynamics::ErrorCode::SOCKET_BIND_FAILED,
			             "Failed to bind configReceiver to port " << space_docking::network::vde_net::kDefaultConfigPort);
		}
		else
		{
			SD_LOG_INFO("Config receiver active on port " << space_docking::network::vde_net::kDefaultConfigPort);
		}
	}

	std::signal(SIGINT, signalHandler);

	std::cout << "Starting 1 ms cyclic simulation loop with UDP streaming on port "
	          << space_docking::network::vde_net::kDefaultStatePort << ". Press Ctrl+C to terminate." << std::endl;

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
			config.chaserMass = cfgPkt.chaserMassKg;
			config.targetMass = cfgPkt.targetMassKg;
			config.chaserInertia.m = cfgPkt.chaserInertia.m;
			config.targetInertia.m = cfgPkt.targetInertia.m;
			config.frameMapping = (cfgPkt.frameMapping == 1) ? space_docking::dynamics::FrameMapping::FLIPPED : space_docking::dynamics::FrameMapping::DEFAULT;
			engine.initialize(config);
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

			// VDE Stewart Platform hardware representation
			const auto relEuler = rel.toEuler();
			pkt.vde.positionM = { rel.pose.position.x, rel.pose.position.y, rel.pose.position.z };
			pkt.vde.linearRateMS = { rel.velocity.x, rel.velocity.y, rel.velocity.z };
			pkt.vde.orientationRad = { relEuler.roll, relEuler.pitch, relEuler.yaw };
			pkt.vde.orientationRateRadS = { chaserState.angularRate.x - targetState.angularRate.x,
			                               chaserState.angularRate.y - targetState.angularRate.y,
			                               chaserState.angularRate.z - targetState.angularRate.z };
			// ForceTorque: torque first, force second!
			pkt.vde.moving = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
			pkt.vde.overhead = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };

			pkt.ngcChaser = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
			pkt.ngcTarget = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };

			pkt.disturbancesOn = 1;
			std::snprintf(pkt.ngcMode, sizeof(pkt.ngcMode), "FINAL APPROACH (closing)");
			std::snprintf(pkt.ngcFailureCase, sizeof(pkt.ngcFailureCase), "None (nominal)");

			stateSender.sendTo(&pkt, sizeof(pkt), space_docking::network::vde_net::kDefaultHost, space_docking::network::vde_net::kDefaultStatePort);
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

		// High-precision wait to maintain exact 1 ms (1000 Hz) cyclic rate
		while (std::chrono::steady_clock::now() - tickStart < std::chrono::microseconds(1000))
		{
			std::this_thread::yield();
		}
	}

	std::cout << "\nDynamics Engine cleanly stopped after " << stepCount << " cycles (" << (stepCount * dt) << " seconds)." << std::endl;
	engine.flushLogs();

#if defined(_WIN32)
	timeEndPeriod(1);
#endif
	return 0;
}
