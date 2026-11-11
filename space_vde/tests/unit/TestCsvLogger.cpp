/**
 * @file TestCsvLogger.cpp
 * @brief Unit tests for asynchronous CsvLogger and 10 ms decimation TelemetryAccumulator.
 *
 * Verifies:
 * 1. TelemetryAccumulator correctly samples 1 ms physics steps and triggers isReady() at 10 ms.
 * 2. CsvLogger writes streamlined 25-column header and asynchronously persists trajectory rows.
 * 3. DynamicsEngine correctly feeds TelemetryAccumulator and writes simulation_trajectory.csv.
 */

#include "space_docking/dynamics_engine/CsvLogger.hpp"
#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/dynamics_engine/Types.hpp"
#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

// Test assertion macro that always executes even in Release (-DNDEBUG) builds
#define REQUIRE(cond) \
	do { \
		if (!(cond)) { \
			std::cerr << "FAILED REQUIRE(" #cond ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
			std::exit(1); \
		} \
	} while (false)

int main()
{
	using namespace space_docking::dynamics;

	std::cout << "Running CsvLogger and TelemetryAccumulator Unit Tests...\n";
	std::cout << "Working directory: " << std::filesystem::current_path().string() << "\n";

	// =========================================================================
	// 1. Test TelemetryAccumulator: 10 ms decimation window and true net state
	// =========================================================================
	{
		std::cout << "  Testing TelemetryAccumulator 10 ms decimation..." << std::endl;
		TelemetryAccumulator accum(10);
		REQUIRE(accum.getDecimation() == 10);
		REQUIRE(accum.getSampleCount() == 0);
		REQUIRE(!accum.isReady());

		Pose startPose{};
		startPose.position = Vector3{0.0, 0.0, 0.0};
		startPose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};
		accum.reset(startPose);

		// Feed 9 1-ms samples (accumulating, not yet ready)
		for (int i = 1; i <= 9; ++i)
		{
			BodyState chaser{};
			chaser.pose.position.x = 0.1 * i;
			BodyState target{};
			RelativeState rel{};
			rel.pose.position.x = 0.1 * i;
			rel.velocity.x = 0.5;
			rel.pose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};

			accum.addSample(chaser, target, rel);
			REQUIRE(accum.getSampleCount() == static_cast<uint32_t>(i));
			REQUIRE(!accum.isReady());
		}

		// Feed 10th sample (window is now complete)
		{
			BodyState chaser{};
			chaser.pose.position.x = 1.0;
			BodyState target{};
			RelativeState rel{};
			rel.pose.position.x = 1.0;
			rel.velocity.x = 0.5;
			rel.pose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};

			accum.addSample(chaser, target, rel);
			REQUIRE(accum.getSampleCount() == 10);
			REQUIRE(accum.isReady());

			// Compute net entry at the 10th ms
			const TelemetryLogEntry entry = accum.computeNetEntryAndAdvance(0.010, 10);
			REQUIRE(entry.cycleCount == 10);
			REQUIRE(std::abs(entry.timestamp - 0.010) < 1e-6);

			// Chaser and Relative positions at sample 10 should be exactly 1.0 m
			REQUIRE(std::abs(entry.chaserPose.position.x - 1.0) < 1e-6);
			REQUIRE(std::abs(entry.relativePose.position.x - 1.0) < 1e-6);
			REQUIRE(std::abs(entry.relativeVelocity.x - 0.5) < 1e-6);

			// After advance, accumulator resets for the next 10 ms window
			REQUIRE(accum.getSampleCount() == 0);
			REQUIRE(!accum.isReady());
		}
	}

	// =========================================================================
	// 2. Test Async CsvLogger file creation and background worker writing
	// =========================================================================
	{
		std::cout << "  Testing Async CsvLogger disk persistence..." << std::endl;
		const std::string testFile = "logs/test_trajectory_output.csv";

		CsvLogger logger;
		ErrorCode ecOpen = logger.open(testFile);
		REQUIRE(ecOpen == ErrorCode::OK);
		REQUIRE(logger.isOpen());

		TelemetryLogEntry entry{};
		entry.timestamp = 0.010;
		entry.cycleCount = 10;
		entry.chaserPose.position = Vector3{1.23, 4.56, 7.89};
		entry.chaserPose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};
		entry.targetPose.position = Vector3{0.0, 0.0, 0.0};
		entry.targetPose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};
		entry.relativePose.position = Vector3{1.23, 4.56, 7.89};
		entry.relativePose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};
		entry.relativeVelocity = Vector3{0.05, 0.0, 0.0};

		REQUIRE(logger.log(entry) == ErrorCode::OK);

		// Write a generic row (SDD Figure 3 backwards compatibility)
		REQUIRE(logger.writeRow({0.020, 20.0, 9.99}) == ErrorCode::OK);

		logger.flush();
		REQUIRE(logger.getWrittenRowCount() >= 2);

		logger.close();
		REQUIRE(!logger.isOpen());

		// Verify file contents on disk
		std::ifstream inFile(testFile);
		REQUIRE(inFile.is_open());

		std::string header;
		std::getline(inFile, header);
		REQUIRE(!header.empty());
		REQUIRE(header.find("timestamp_s") != std::string::npos);
		REQUIRE(header.find("chaser_pos_x_m") != std::string::npos);
		REQUIRE(header.find("target_pos_x_m") != std::string::npos);
		REQUIRE(header.find("rel_pos_x_m") != std::string::npos);

		std::string row1;
		std::getline(inFile, row1);
		REQUIRE(!row1.empty());
		REQUIRE(row1.find("1.230000") != std::string::npos);

		std::string row2;
		std::getline(inFile, row2);
		REQUIRE(!row2.empty());
		REQUIRE(row2.find("9.990000") != std::string::npos);

		inFile.close();
	}

	// =========================================================================
	// 3. Test Full DynamicsEngine Integration with 10 ms Decimated CSV Logging
	// =========================================================================
	{
		std::cout << "  Testing DynamicsEngine integration with CsvLogger..." << std::endl;
		RunConfig config{};

		// Look for simulation_config.xml relative to working dir, parent dir, or space_vde/
		std::string configPath = "config/simulation_config.xml";
		if (!std::filesystem::exists(configPath))
		{
			if (std::filesystem::exists("../config/simulation_config.xml"))
			{
				configPath = "../config/simulation_config.xml";
			}
			else if (std::filesystem::exists("space_vde/config/simulation_config.xml"))
			{
				configPath = "space_vde/config/simulation_config.xml";
			}
		}

		REQUIRE(XmlConfigLoader::loadRunConfig(configPath, config) == ErrorCode::OK);

		config.loggingEnabled = true;
		config.logDirectory = "logs";
		config.logFileName = "test_engine_trajectory.csv";
		config.logDecimation = 10;

		const std::string engineCsv = "logs/test_engine_trajectory.csv";

		DynamicsEngine engine;
		REQUIRE(engine.initialize(config) == ErrorCode::OK);
		REQUIRE(engine.getCsvLogger().isOpen());

		// Step for 25 ms (should produce exactly 2 decimated log entries: at 10 ms and 20 ms)
		constexpr double dt = 0.001;
		for (int i = 0; i < 25; ++i)
		{
			REQUIRE(engine.step(dt) == ErrorCode::OK);
		}

		REQUIRE(engine.getStepCount() == 25);
		REQUIRE(std::abs(engine.getSimTime() - 0.025) < 1e-6);

		engine.flushLogs();

		// Verify CSV file exists and has 1 header row + 2 data rows = 3 rows
		std::ifstream inFile(engineCsv);
		REQUIRE(inFile.is_open());

		std::string line;
		int rowCount = 0;
		while (std::getline(inFile, line))
		{
			if (!line.empty())
			{
				rowCount++;
			}
		}
		inFile.close();

		REQUIRE(rowCount == 3);
	}

	std::cout << "[PASS] All CsvLogger and TelemetryAccumulator tests passed successfully!\n";
	return 0;
}
