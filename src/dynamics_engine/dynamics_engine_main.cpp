#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"
#include "space_docking/utils/Logger.hpp"
#include "space_docking/utils/MathUtils.hpp"

#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

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
	space_docking::dynamics::ErrorCode ec = space_docking::dynamics::XmlConfigLoader::loadRunConfig(configPath, config);
	if (ec != space_docking::dynamics::ErrorCode::OK)
	{
		std::cerr << "[ERROR] Failed to load configuration from " << configPath << "\n";
		return 1;
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
		std::cerr << "[ERROR] Engine initialization failed with code: " << static_cast<int>(ec) << std::endl;
		return 1;
	}

	std::signal(SIGINT, signalHandler);

	std::cout << "Starting 1 ms cyclic simulation loop. Press Ctrl+C to terminate." << std::endl;

	uint64_t stepCount = 0;
	constexpr double dt = 0.001; // 1 ms physics period

	while (g_running)
	{
		const auto tickStart = std::chrono::steady_clock::now();

		engine.step(dt);
		stepCount++;

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

#if defined(_WIN32)
	timeEndPeriod(1);
#endif
	return 0;
}
