#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"
#include "space_docking/utils/Logger.hpp"

#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

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
	std::string configPath = "config/simulation_config.xml";
	if (argc > 1)
	{
		configPath = argv[1];
	}

	std::cout << "========================================================\n";
	std::cout << " Space Docking Dynamics Engine (GST/FST HIL Simulation) \n";
	std::cout << "========================================================\n";
	std::cout << "Loading configuration: " << configPath << "\n";

	space_docking::dynamics::RunConfig config{};
	space_docking::dynamics::ErrorCode ec = space_docking::dynamics::XmlConfigLoader::loadRunConfig(configPath, config);
	if (ec != space_docking::dynamics::ErrorCode::OK)
	{
		std::cerr << "[ERROR] Failed to load configuration from " << configPath << "\n";
		return 1;
	}

	std::cout << "Chaser Mass: " << config.chaserMass << " kg\n";
	std::cout << "Target Mass: " << config.targetMass << " kg\n";

	space_docking::dynamics::DynamicsEngine engine;
	ec = engine.initialize(config);
	if (ec != space_docking::dynamics::ErrorCode::OK)
	{
		std::cerr << "[ERROR] Engine initialization failed with code: " << static_cast<int>(ec) << "\n";
		return 1;
	}

	std::signal(SIGINT, signalHandler);

	std::cout << "Starting 1 ms cyclic simulation loop. Press Ctrl+C to terminate.\n";

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
			          << euler.yaw << " rad\n";
		}

		// Microsecond-accurate sleep to maintain 1 ms rate
		const auto elapsed = std::chrono::steady_clock::now() - tickStart;
		const auto sleepTime = std::chrono::microseconds(1000) - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
		if (sleepTime.count() > 0)
		{
			std::this_thread::sleep_for(sleepTime);
		}
	}

	std::cout << "\nDynamics Engine cleanly stopped after " << stepCount << " cycles (" << (stepCount * dt) << " seconds).\n";
	return 0;
}
