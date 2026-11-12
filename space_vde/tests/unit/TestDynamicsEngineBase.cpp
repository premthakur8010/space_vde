#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/dynamics_engine/RelativeKinematics.hpp"
#include "space_docking/dynamics_engine/SpacecraftDynamics.hpp"
#include "space_docking/dynamics_engine/Types.hpp"
#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"
#include "space_docking/dynamics_engine/XmlConfigUtils.hpp"
#include "space_docking/utils/MathUtils.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

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

	std::cout << "Testing Dynamics Engine Base Classes...\n";

	// 1. Test SpacecraftDynamics setup and move
	SpacecraftDynamics chaser;
	Matrix3 inertia;
	inertia.m[0] = 50.0;
	inertia.m[4] = 50.0;
	inertia.m[8] = 25.0;

	REQUIRE(chaser.setup(500.0, inertia) == ErrorCode::OK);

	ForceTorque ft;
	ft.force = Vector3{10.0, 0.0, 0.0}; // 10 N in X
	ft.torque = Vector3{0.0, 0.0, 0.0};

	BodyState state = chaser.move(ft, 0.001); // 1 ms step
	REQUIRE(state.velocity.x > 0.0);
	REQUIRE(state.pose.position.x > 0.0);

	// 2. Test RelativeKinematics
	BodyState targetState{};
	targetState.pose.position = Vector3{10.0, 0.0, 0.0};
	targetState.pose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0}; // w=1, x=0, y=0, z=0

	BodyState chaserState{};
	chaserState.pose.position = Vector3{5.0, 0.0, 0.0};
	chaserState.pose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};

	RelativeKinematics relKin;
	RelativeState rel = relKin.compute(chaserState, targetState);
	REQUIRE(rel.pose.position.x == -5.0); // 5 - 10 = -5
	REQUIRE(rel.pose.orientation.w == 1.0);

	EulerAngles euler = rel.toEuler();
	REQUIRE(std::abs(euler.yaw) < 1e-6);
	REQUIRE(std::abs(euler.pitch) < 1e-6);
	REQUIRE(std::abs(euler.roll) < 1e-6);

	// 3. Test Dynamic Parameter Updates & Mounting Frame Mapping
	DynamicsEngine engineTest;
	RunConfig testCfg{};
	testCfg.chaserMass = 100.0;
	testCfg.targetMass = 200.0;
	REQUIRE(engineTest.initialize(testCfg) == ErrorCode::OK);
	REQUIRE(engineTest.getConfig().frameMapping == FrameMapping::DEFAULT);

	engineTest.updateParameters(150.0, inertia, 250.0, inertia, FrameMapping::FLIPPED);
	REQUIRE(engineTest.getConfig().frameMapping == FrameMapping::FLIPPED);
	REQUIRE(engineTest.getChaser().getMass() == 150.0);
	REQUIRE(engineTest.getTarget().getMass() == 250.0);

	// 4. Test XmlConfigLoader and shared XML path resolution
	const std::string configPath = xml_utils::resolveConfigFilePath("simulation_config.xml");
	REQUIRE(std::filesystem::exists(configPath));
	REQUIRE(std::filesystem::exists(xml_utils::resolveConfigFilePath("default.xml")));
	REQUIRE(xml_utils::resolveConfigFilePath("missing_file.xml") == "config/missing_file.xml");

	RunConfig xmlConfig{};
	ErrorCode xmlEc = XmlConfigLoader::loadRunConfig(configPath, xmlConfig);
	REQUIRE(xmlEc == ErrorCode::OK);
	REQUIRE(xmlConfig.chaserMass == 50.0);
	REQUIRE(xmlConfig.targetMass == 100.0);
	REQUIRE(xmlConfig.chaserInertia.m[0] == 5.0);
	REQUIRE(xmlConfig.targetHost == "127.0.0.1");
	REQUIRE(xmlConfig.configPort == 5001);
	REQUIRE(xmlConfig.statePort == 5002);

	// 5. Test DynamicsEngine coordinator with XML config
	DynamicsEngine engine;
	REQUIRE(engine.initialize(xmlConfig) == ErrorCode::OK);
	REQUIRE(engine.getChaser().getMass() == 50.0);
	REQUIRE(engine.getTarget().getMass() == 100.0);
	REQUIRE(engine.step(0.001) == ErrorCode::OK);

	// 6. Test <network> XML parsing, comment stripping, and invalid port rejection
	const std::string rawNetXml =
		"<simulationconfig>"
		"<!-- <network><targethost>10.0.0.1</targethost><configport>9999</configport></network> -->"
		"<network><targethost> 192.168.1.42 </targethost><configport>6101</configport><stateport>6102</stateport></network>"
		"</simulationconfig>";
	std::string parsedHost = "127.0.0.1";
	uint16_t parsedConfigPort = 5001;
	uint16_t parsedStatePort = 5002;
	std::string netErr;
	REQUIRE(xml_utils::parseNetworkBlock(xml_utils::stripXmlComments(rawNetXml),
	                                     parsedHost,
	                                     parsedConfigPort,
	                                     parsedStatePort,
	                                     netErr));
	REQUIRE(parsedHost == "192.168.1.42");
	REQUIRE(parsedConfigPort == 6101);
	REQUIRE(parsedStatePort == 6102);

	const std::string badPortZero = "<network><configport>0</configport></network>";
	REQUIRE(!xml_utils::parseNetworkBlock(badPortZero, parsedHost, parsedConfigPort, parsedStatePort, netErr));
	REQUIRE(!netErr.empty());

	const std::string badPortRange = "<network><stateport>70000</stateport></network>";
	netErr.clear();
	REQUIRE(!xml_utils::parseNetworkBlock(badPortRange, parsedHost, parsedConfigPort, parsedStatePort, netErr));
	REQUIRE(!netErr.empty());

	const std::string badPortText = "<network><configport>abc</configport></network>";
	netErr.clear();
	REQUIRE(!xml_utils::parseNetworkBlock(badPortText, parsedHost, parsedConfigPort, parsedStatePort, netErr));
	REQUIRE(!netErr.empty());

	// 7. Test XmlConfigLoader::loadRunConfig reading <network> and HIL tags from an XML file
	const std::string tempXmlPath = "test_network_override.xml";
	{
		std::ofstream tmp(tempXmlPath);
		tmp << "<simulationconfig>"
		    << "<chaser><masskg>120.0</masskg></chaser>"
		    << "<target><masskg>240.0</masskg></target>"
		    << "<network>"
		    << "<targethost>127.0.0.1</targethost><configport>7101</configport><stateport>7102</stateport>"
		    << "<hardwareenabled>false</hardwareenabled><hardwarehost>10.0.0.5</hardwarehost>"
		    << "<hardwarecommandport>6006</hardwarecommandport><hardwaretelemetryport>6005</hardwaretelemetryport>"
		    << "</network>"
		    << "</simulationconfig>\n";
	}
	RunConfig netFileConfig{};
	REQUIRE(XmlConfigLoader::loadRunConfig(tempXmlPath, netFileConfig) == ErrorCode::OK);
	REQUIRE(netFileConfig.targetHost == "127.0.0.1");
	REQUIRE(netFileConfig.configPort == 7101);
	REQUIRE(netFileConfig.statePort == 7102);
	REQUIRE(netFileConfig.hardwareEnabled == false);
	REQUIRE(netFileConfig.hardwareHost == "10.0.0.5");
	REQUIRE(netFileConfig.hardwareCommandPort == 6006);
	REQUIRE(netFileConfig.hardwareTelemetryPort == 6005);
	REQUIRE(xml_utils::resolveConfigFilePath("simulation_config.xml", tempXmlPath) == tempXmlPath);
	std::remove(tempXmlPath.c_str());

	// 8. Test invalid HIL hardware tags rejection & DynamicsEngine telemetry getters
	bool hwEnabled = true;
	std::string hwHost = "127.0.0.1";
	uint16_t hwCmdPort = 5006;
	uint16_t hwTelPort = 5005;
	netErr.clear();
	REQUIRE(!xml_utils::parseHardwareNetworkBlock("<network><hardwareenabled>maybe</hardwareenabled></network>",
	                                              hwEnabled, hwHost, hwCmdPort, hwTelPort, netErr));
	netErr.clear();
	REQUIRE(!xml_utils::parseHardwareNetworkBlock("<network><hardwarecommandport>0</hardwarecommandport></network>",
	                                              hwEnabled, hwHost, hwCmdPort, hwTelPort, netErr));

	Telemetry hilTel{};
	hilTel.timestamp = 0.5;
	hilTel.groundStewartPlatformForces = { {0.1, 0.2, 0.3}, {-15.0, 0.0, 0.0} };
	engine.setTelemetry(hilTel);
	REQUIRE(space_docking::utils::nearlyEqual(engine.getTelemetry().timestamp, 0.5));
	REQUIRE(space_docking::utils::nearlyEqual(engine.getTelemetry().groundStewartPlatformForces.force.x, -15.0));

	GspFeedback gspFb{};
	gspFb.legLengths = { 0.41, 0.42, 0.43, 0.44, 0.45, 0.46 };
	engine.setGspFeedback(gspFb);
	REQUIRE(space_docking::utils::nearlyEqual(engine.getGspFeedback().legLengths[5], 0.46));

	std::cout << "[PASS] TestDynamicsEngineBase completed successfully.\n";
	return 0;
}
