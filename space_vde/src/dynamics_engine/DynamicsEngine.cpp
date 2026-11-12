/**
 * @file DynamicsEngine.cpp
 * @brief Implementation of the central Dynamics Engine physics coordinator.
 *
 * Coordinates the execution of Chaser and Target 6-DOF dynamics, relative docking kinematics,
 * platform pose computation, and asynchronous CSV telemetry logging at 1 kHz frequency.
 */

#include "space_docking/dynamics_engine/DynamicsEngine.hpp"
#include "space_docking/utils/LogMacros.hpp"

namespace space_docking::dynamics
{
	// =========================================================================
	// 1. INITIALIZATION: Loading Configuration & Starting Simulation Run
	// =========================================================================

	// Sets up spacecraft masses, initial positions/velocities from XML, and opens CSV logging
	ErrorCode DynamicsEngine::initialize(const RunConfig& config)
	{
		m_config = config;

		// 1. Configure Chaser physical mass (kg) and moment of inertia tensor (kg*m^2)
		ErrorCode ec = m_chaser.setup(config.chaserMass, config.chaserInertia);
		if (ec != ErrorCode::OK)
		{
			SD_LOG_ERROR(ec, "Failed to setup Chaser dynamics parameters");
			return ec;
		}

		// 2. Configure Target physical mass (kg) and moment of inertia tensor (kg*m^2)
		ec = m_target.setup(config.targetMass, config.targetInertia);
		if (ec != ErrorCode::OK)
		{
			SD_LOG_ERROR(ec, "Failed to setup Target dynamics parameters");
			return ec;
		}

		// 3. Set initial state vectors from XML configuration
		BodyState chaserState{};
		chaserState.pose = config.initialSeparation;
		chaserState.velocity = config.chaserInitialVelocity;
		m_chaser.setState(chaserState);

		BodyState targetState{};
		targetState.velocity = config.targetInitialVelocity;
		m_target.setState(targetState);

		// 4. Compute initial relative kinematics and platform poses
		m_latestRelativeState = m_relativeKinematics.compute(chaserState, targetState);
		m_latestPoses = m_poseComputation.compute(chaserState, targetState, m_latestGspFeedback);
		m_initialized = true;

		// 5. Initialize CSV telemetry logger and decimation accumulator (10 ms = 100 Hz log rate)
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

	// =========================================================================
	// 2. DYNAMIC PARAMETER UPDATE (Safe Mid-Run Updates via UDP)
	// =========================================================================

	// Safely updates masses and inertias mid-run via UDP without resetting positions or desyncing clocks
	ErrorCode DynamicsEngine::updateParameters(double chaserMass, const Matrix3& chaserInertia,
	                                           double targetMass, const Matrix3& targetInertia,
	                                           FrameMapping frameMapping)
	{
		ErrorCode ec = m_chaser.setup(chaserMass, chaserInertia);
		if (ec != ErrorCode::OK)
		{
			SD_LOG_ERROR(ec, "Failed to update Chaser dynamics parameters");
			return ec;
		}

		ec = m_target.setup(targetMass, targetInertia);
		if (ec != ErrorCode::OK)
		{
			SD_LOG_ERROR(ec, "Failed to update Target dynamics parameters");
			return ec;
		}

		m_config.chaserMass = chaserMass;
		m_config.chaserInertia = chaserInertia;
		m_config.targetMass = targetMass;
		m_config.targetInertia = targetInertia;
		m_config.frameMapping = frameMapping;

		SD_LOG_INFO("DynamicsEngine parameters dynamically updated without resetting trajectory state");
		return ErrorCode::OK;
	}

	// =========================================================================
	// 3. STEP: The 1 kHz Real-Time Physics Tick (dt = 0.001 s)
	// =========================================================================

	// The core 1000 Hz physics tick: routes sensor forces, integrates 6-DOF motion, and logs decimated telemetry
	ErrorCode DynamicsEngine::step(double dt)
	{
		if (!m_initialized)
		{
			SD_LOG_ERROR(ErrorCode::NOT_INITIALIZED, "DynamicsEngine::step() invoked before initialize()");
			return ErrorCode::NOT_INITIALIZED;
		}

		// ---------------------------------------------------------------------
		// Step 3.1: Telemetry Channel Routing by Mounting Mode
		// ---------------------------------------------------------------------
		// In the physical test facility:
		// - DEFAULT: Chaser mechanism is mounted on Ground Stewart Platform (GSP),
		//            Target mechanism is mounted on Flight Stewart Platform (FSP/overhead).
		// - FLIPPED: Target is mounted on GSP, Chaser is mounted on FSP.
		const ForceTorque chaserSensorFT = (m_config.frameMapping == FrameMapping::DEFAULT)
		    ? m_currentTelemetry.groundStewartPlatformForces
		    : m_currentTelemetry.flightStewartPlatformForces;

		const ForceTorque targetSensorFT = (m_config.frameMapping == FrameMapping::DEFAULT)
		    ? m_currentTelemetry.flightStewartPlatformForces
		    : m_currentTelemetry.groundStewartPlatformForces;

		// ---------------------------------------------------------------------
		// Step 3.2: Combine Sensor Forces with Guidance/Thruster Commands
		// ---------------------------------------------------------------------
		// Total Force = Sensor Contact Loads + Active Navigation/Thruster Forces
		ForceTorque chaserTotalFT = chaserSensorFT;
		chaserTotalFT.force.x += m_ngcForces[0].force.x;
		chaserTotalFT.force.y += m_ngcForces[0].force.y;
		chaserTotalFT.force.z += m_ngcForces[0].force.z;
		chaserTotalFT.torque.x += m_ngcForces[0].torque.x;
		chaserTotalFT.torque.y += m_ngcForces[0].torque.y;
		chaserTotalFT.torque.z += m_ngcForces[0].torque.z;

		ForceTorque targetTotalFT = targetSensorFT;
		targetTotalFT.force.x += m_ngcForces[1].force.x;
		targetTotalFT.force.y += m_ngcForces[1].force.y;
		targetTotalFT.force.z += m_ngcForces[1].force.z;
		targetTotalFT.torque.x += m_ngcForces[1].torque.x;
		targetTotalFT.torque.y += m_ngcForces[1].torque.y;
		targetTotalFT.torque.z += m_ngcForces[1].torque.z;

		// ---------------------------------------------------------------------
		// Step 3.3: 6-DOF Numerical Propagation (Newton-Euler via Symplectic Euler)
		// ---------------------------------------------------------------------
		const BodyState chaserState = m_chaser.move(chaserTotalFT, dt);
		const BodyState targetState = m_target.move(targetTotalFT, dt);

		// ---------------------------------------------------------------------
		// Step 3.4: Compute Hardware Platform Poses (Blending Encoders if Active)
		// ---------------------------------------------------------------------
		m_latestPoses = m_poseComputation.compute(chaserState, targetState, m_latestGspFeedback);

		// ---------------------------------------------------------------------
		// Step 3.5: Relative Docking Kinematics (r_rel, v_rel, q_rel, qDot_rel)
		// ---------------------------------------------------------------------
		m_latestRelativeState = m_relativeKinematics.compute(chaserState, targetState);

		m_stepCount++;
		m_simTime += dt;

		// ---------------------------------------------------------------------
		// Step 3.6: Asynchronous Decimated CSV Logging (Every 10 ms = 100 Hz)
		// ---------------------------------------------------------------------
		// Batching samples and writing in the background prevents slow disk I/O
		// from causing timing jitter in this critical 1 ms real-time physics loop.
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

	// =========================================================================
	// 4. State Accessors and Sensor Mutators
	// =========================================================================

	// Injects live load-cell force/torque readings from the Stewart platform
	void DynamicsEngine::setTelemetry(const Telemetry& t) noexcept
	{
		m_currentTelemetry = t;
	}

	// Injects guidance thruster forces for Chaser [0] and Target [1]
	void DynamicsEngine::setNgcForces(const std::array<ForceTorque, 2>& f) noexcept
	{
		m_ngcForces = f;
	}

	// Injects measured optical encoder displacement from the physical Stewart platform
	void DynamicsEngine::setGspFeedback(const GspFeedback& f) noexcept
	{
		m_latestGspFeedback = f;
	}

	const Telemetry& DynamicsEngine::getTelemetry() const noexcept
	{
		return m_currentTelemetry;
	}

	const GspFeedback& DynamicsEngine::getGspFeedback() const noexcept
	{
		return m_latestGspFeedback;
	}

	// Returns read-only reference to the Chaser spacecraft dynamics object
	const SpacecraftDynamics& DynamicsEngine::getChaser() const noexcept
	{
		return m_chaser;
	}

	// Returns read-only reference to the Target spacecraft dynamics object
	const SpacecraftDynamics& DynamicsEngine::getTarget() const noexcept
	{
		return m_target;
	}

	// Returns the latest 14-element relative docking state (r_rel, v_rel, q_rel, qDot_rel)
	const RelativeState& DynamicsEngine::getRelativeState() const noexcept
	{
		return m_latestRelativeState;
	}

	// Returns the computed 3D platform poses [0]=Chaser, [1]=Target
	const std::array<Pose, 2>& DynamicsEngine::getPoses() const noexcept
	{
		return m_latestPoses;
	}

	// Forces all buffered in-memory CSV telemetry rows to be written to disk immediately
	void DynamicsEngine::flushLogs() noexcept
	{
		m_csvLogger.flush();
	}

	// Returns non-const reference to the internal asynchronous CSV telemetry logger
	CsvLogger& DynamicsEngine::getCsvLogger() noexcept
	{
		return m_csvLogger;
	}

	// Returns const reference to the internal asynchronous CSV telemetry logger
	const CsvLogger& DynamicsEngine::getCsvLogger() const noexcept
	{
		return m_csvLogger;
	}

	// Returns the total number of 1 ms integration steps completed
	uint64_t DynamicsEngine::getStepCount() const noexcept
	{
		return m_stepCount;
	}

	// Returns elapsed simulation time in seconds (stepCount * dt)
	double DynamicsEngine::getSimTime() const noexcept
	{
		return m_simTime;
	}
}
