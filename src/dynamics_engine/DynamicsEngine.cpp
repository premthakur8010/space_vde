/**
 * @file DynamicsEngine.cpp
 * @brief Implementation of the central Dynamics Engine physics coordinator.
 */

#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/utils/LogMacros.hpp"

namespace space_docking::dynamics
{
	ErrorCode DynamicsEngine::initialize(const RunConfig& config)
	{
		m_config = config;

		// Setup Chaser dynamics from XML configuration
		ErrorCode ec = m_chaser.setup(config.chaserMass, config.chaserInertia);
		if (ec != ErrorCode::OK)
		{
			SD_LOG_ERROR(ec, "Failed to setup Chaser dynamics parameters");
			return ec;
		}

		// Setup Target dynamics from XML configuration
		ec = m_target.setup(config.targetMass, config.targetInertia);
		if (ec != ErrorCode::OK)
		{
			SD_LOG_ERROR(ec, "Failed to setup Target dynamics parameters");
			return ec;
		}

		// Setup initial state from configuration
		BodyState chaserState{};
		chaserState.pose = config.initialSeparation;
		chaserState.velocity = config.chaserInitialVelocity;
		m_chaser.setState(chaserState);

		BodyState targetState{};
		targetState.velocity = config.targetInitialVelocity;
		m_target.setState(targetState);

		m_frameTransform.setFlipMode(config.frameMapping);
		m_latestRelativeState = m_relativeKinematics.compute(chaserState, targetState);
		m_latestPoses = m_poseComputation.compute(chaserState, targetState, m_latestGspFeedback);
		m_initialized = true;

		// Initialize CSV telemetry logger and decimation accumulator
		m_stepCount = 0;
		m_simTime = 0.0;
		m_accumulator = TelemetryAccumulator(config.logDecimation);
		m_accumulator.reset(m_latestRelativeState.pose);

		if (config.loggingEnabled && !config.logFileName.empty())
		{
			std::string fullPath = config.logDirectory;
			if (!fullPath.empty() && fullPath.back() != '/' && fullPath.back() != '\\')
			{
				fullPath += "/";
			}
			fullPath += config.logFileName;

			const ErrorCode ecLog = m_csvLogger.open(fullPath);
			if (ecLog != ErrorCode::OK)
			{
				SD_LOG_ERROR(ecLog, "Failed to open CSV trajectory log: " << fullPath);
			}
			else
			{
				SD_LOG_INFO("CSV telemetry logger active: " << fullPath << " (rate: every " << config.logDecimation << " ms)");
			}
		}

		SD_LOG_INFO("DynamicsEngine successfully initialized with 2 spacecraft bodies");
		return ErrorCode::OK;
	}

	ErrorCode DynamicsEngine::step(double dt)
	{
		if (!m_initialized)
		{
			SD_LOG_ERROR(ErrorCode::NOT_INITIALIZED, "DynamicsEngine::step() invoked before initialize()");
			return ErrorCode::NOT_INITIALIZED;
		}

		// 1. Retrieve & convert external sensor forces into body frames
		const auto bodyForces = m_frameTransform.convertForces(m_currentTelemetry);

		// 2. Add external guidance/control forces per body
		ForceTorque chaserTotalFT = bodyForces[0];
		chaserTotalFT.force.x += m_ngcForces[0].force.x;
		chaserTotalFT.force.y += m_ngcForces[0].force.y;
		chaserTotalFT.force.z += m_ngcForces[0].force.z;
		chaserTotalFT.torque.x += m_ngcForces[0].torque.x;
		chaserTotalFT.torque.y += m_ngcForces[0].torque.y;
		chaserTotalFT.torque.z += m_ngcForces[0].torque.z;

		ForceTorque targetTotalFT = bodyForces[1];
		targetTotalFT.force.x += m_ngcForces[1].force.x;
		targetTotalFT.force.y += m_ngcForces[1].force.y;
		targetTotalFT.force.z += m_ngcForces[1].force.z;
		targetTotalFT.torque.x += m_ngcForces[1].torque.x;
		targetTotalFT.torque.y += m_ngcForces[1].torque.y;
		targetTotalFT.torque.z += m_ngcForces[1].torque.z;

		// 3. Propagate both spacecraft bodies
		const BodyState chaserState = m_chaser.move(chaserTotalFT, dt);
		const BodyState targetState = m_target.move(targetTotalFT, dt);

		// 4. Compute spacecraft poses
		m_latestPoses = m_poseComputation.compute(chaserState, targetState, m_latestGspFeedback);

		// 5. Compute relative docking state Y
		m_latestRelativeState = m_relativeKinematics.compute(chaserState, targetState);

		m_stepCount++;
		m_simTime += dt;

		// 6. Accumulate and log telemetry asynchronously for 3D visualization replay (every 10 ms)
		if (m_config.loggingEnabled && m_csvLogger.isOpen())
		{
			m_accumulator.addSample(chaserState, targetState, m_latestRelativeState);

			if (m_accumulator.isReady())
			{
				const TelemetryLogEntry entry = m_accumulator.computeNetEntryAndAdvance(m_simTime, m_stepCount);
				m_csvLogger.log(entry);
			}
		}

		return ErrorCode::OK;
	}

	void DynamicsEngine::setTelemetry(const Telemetry& t) noexcept
	{
		m_currentTelemetry = t;
	}

	void DynamicsEngine::setNgcForces(const std::array<ForceTorque, 2>& f) noexcept
	{
		m_ngcForces = f;
	}

	void DynamicsEngine::setGspFeedback(const GspFeedback& f) noexcept
	{
		m_latestGspFeedback = f;
	}

	const SpacecraftDynamics& DynamicsEngine::getChaser() const noexcept
	{
		return m_chaser;
	}

	const SpacecraftDynamics& DynamicsEngine::getTarget() const noexcept
	{
		return m_target;
	}

	const RelativeState& DynamicsEngine::getRelativeState() const noexcept
	{
		return m_latestRelativeState;
	}

	const std::array<Pose, 2>& DynamicsEngine::getPoses() const noexcept
	{
		return m_latestPoses;
	}

	void DynamicsEngine::flushLogs() noexcept
	{
		m_csvLogger.flush();
	}

	CsvLogger& DynamicsEngine::getCsvLogger() noexcept
	{
		return m_csvLogger;
	}

	const CsvLogger& DynamicsEngine::getCsvLogger() const noexcept
	{
		return m_csvLogger;
	}

	uint64_t DynamicsEngine::getStepCount() const noexcept
	{
		return m_stepCount;
	}

	double DynamicsEngine::getSimTime() const noexcept
	{
		return m_simTime;
	}
}
