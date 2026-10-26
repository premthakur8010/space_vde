#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/dynamics_engine/EngineLogger.hpp"
#include "space_docking/dynamics_engine/FrameTransformation.hpp"
#include "space_docking/dynamics_engine/RelativeKinematics.hpp"
#include "space_docking/dynamics_engine/SpacecraftDynamics.hpp"
#include "space_docking/dynamics_engine/Types.hpp"
#include "space_docking/dynamics_engine/XmlConfigLoader.hpp"

#include <cassert>
#include <iostream>

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

	assert(chaser.setup(500.0, inertia) == ErrorCode::OK);

	ForceTorque ft;
	ft.force = Vector3{10.0, 0.0, 0.0}; // 10 N in X
	ft.torque = Vector3{0.0, 0.0, 0.0};

	BodyState state = chaser.move(ft, 0.001); // 1 ms step
	assert(state.velocity.x > 0.0);
	assert(state.pose.position.x > 0.0);

	// 2. Test RelativeKinematics
	BodyState targetState{};
	targetState.pose.position = Vector3{10.0, 0.0, 0.0};
	targetState.pose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0}; // w=1, x=0, y=0, z=0

	BodyState chaserState{};
	chaserState.pose.position = Vector3{5.0, 0.0, 0.0};
	chaserState.pose.orientation = Quaternion{1.0, 0.0, 0.0, 0.0};

	RelativeKinematics relKin;
	RelativeState rel = relKin.compute(chaserState, targetState);
	assert(rel.pose.position.x == -5.0); // 5 - 10 = -5
	assert(rel.pose.orientation.w == 1.0);

	EulerAngles euler = rel.toEuler();
	assert(std::abs(euler.yaw) < 1e-6);
	assert(std::abs(euler.pitch) < 1e-6);
	assert(std::abs(euler.roll) < 1e-6);

	// 3. Test FrameTransformation
	FrameTransformation ftTransform;
	ftTransform.setFlipMode(FrameMapping::DEFAULT);
	Telemetry tel;
	tel.groundStewartPlatformForces.force = Vector3{1.0, 2.0, 3.0};
	tel.flightStewartPlatformForces.force = Vector3{4.0, 5.0, 6.0};

	auto forces = ftTransform.convertForces(tel);
	assert(forces[0].force.x == 1.0);
	assert(forces[1].force.x == 4.0);

	// Test flip
	ftTransform.setFlipMode(FrameMapping::FLIPPED);
	auto flippedForces = ftTransform.convertForces(tel);
	assert(flippedForces[0].force.x == 4.0);
	assert(flippedForces[1].force.x == 1.0);

	// 4. Test XmlConfigLoader
	RunConfig xmlConfig{};
	ErrorCode xmlEc = XmlConfigLoader::loadRunConfig("config/simulation_config.xml", xmlConfig);
	assert(xmlEc == ErrorCode::OK);
	assert(xmlConfig.chaserMass == 50.0);
	assert(xmlConfig.targetMass == 100.0);
	assert(xmlConfig.chaserInertia.m[0] == 5.0);

	// 5. Test DynamicsEngine coordinator with XML config
	DynamicsEngine engine;
	assert(engine.initialize(xmlConfig) == ErrorCode::OK);
	assert(engine.getChaser().getMass() == 50.0);
	assert(engine.getTarget().getMass() == 100.0);
	assert(engine.step(0.001) == ErrorCode::OK);

	std::cout << "[PASS] TestDynamicsEngineBase completed successfully.\n";
	return 0;
}
