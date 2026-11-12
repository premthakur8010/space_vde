/**
 * @file DynamicsEngine.hpp
 * @brief Central physics simulation coordinator for the Space Docking System.
 *
 * Coordinates execution of Chaser and Target 6-DOF dynamics, relative docking kinematics,
 * and platform pose computations at 1 kHz cycle frequency.
 */

#pragma once

#include <array>
#include <cstdint>

#include "CsvLogger.hpp"
#include "RelativeKinematics.hpp"
#include "SpacecraftDynamics.hpp"
#include "SpacecraftPoseComputation.hpp"
#include "Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Central physics coordinator for the Dynamics Engine.
	 *
	 * PIPELINE IN STEP(dt):
	 * Each 1 ms cycle (1000 Hz) performs:
	 * 1. Force Routing: Resolves load-cell forces based on FrameMapping (DEFAULT vs FLIPPED).
	 * 2. Chaser Dynamics: Integrates 6-DOF translation and attitude via Symplectic Euler (SpacecraftDynamics::move).
	 * 3. Target Dynamics: Integrates Target 6-DOF state via Symplectic Euler (SpacecraftDynamics::move).
	 * 4. Platform Pose Computation: Blends physical testbed encoder feedback if active (SpacecraftPoseComputation::compute).
	 * 5. Relative Kinematics: Computes 14-element state vector Y (RelativeKinematics::compute).
	 * 6. Telemetry Decimation: Buffers high-rate telemetry into TelemetryAccumulator for CSV logging.
	 */
	class DynamicsEngine
	{
	public:
		DynamicsEngine() = default;
		~DynamicsEngine() = default;

		/**
		 * @brief Initialize engine with run configuration parameters loaded from XML.
		 */
		ErrorCode initialize(const RunConfig& config);

		/**
		 * @brief Update physical mass, inertia, and frame mapping dynamically without resetting trajectory.
		 */
		ErrorCode updateParameters(double chaserMass, const Matrix3& chaserInertia,
		                           double targetMass, const Matrix3& targetInertia,
		                           FrameMapping frameMapping);

		/**
		 * @brief Execute one physics cycle step.
		 * @param dt Step size in seconds (e.g. 0.001 s for 1 kHz).
		 */
		ErrorCode step(double dt);

		void setTelemetry(const Telemetry& t) noexcept;
		void setNgcForces(const std::array<ForceTorque, 2>& f) noexcept;
		void setGspFeedback(const GspFeedback& f) noexcept;
		void setGstFeedback(const GspFeedback& f) noexcept { setGspFeedback(f); }

		[[nodiscard]] const RunConfig& getConfig() const noexcept { return m_config; }
		[[nodiscard]] const Telemetry& getTelemetry() const noexcept;
		[[nodiscard]] const GspFeedback& getGspFeedback() const noexcept;
		[[nodiscard]] const SpacecraftDynamics& getChaser() const noexcept;
		[[nodiscard]] const SpacecraftDynamics& getTarget() const noexcept;
		[[nodiscard]] const RelativeState& getRelativeState() const noexcept;
		[[nodiscard]] const std::array<Pose, 2>& getPoses() const noexcept;

		/**
		 * @brief Flush all queued CSV telemetry log entries to disk.
		 */
		void flushLogs() noexcept;

		[[nodiscard]] CsvLogger& getCsvLogger() noexcept;
		[[nodiscard]] const CsvLogger& getCsvLogger() const noexcept;
		[[nodiscard]] uint64_t getStepCount() const noexcept;
		[[nodiscard]] double getSimTime() const noexcept;

	private:
		RunConfig m_config{};
		bool m_initialized{false};

		SpacecraftDynamics m_chaser{};
		SpacecraftDynamics m_target{};

		RelativeKinematics m_relativeKinematics{};
		SpacecraftPoseComputation m_poseComputation{};

		Telemetry m_currentTelemetry{};
		std::array<ForceTorque, 2> m_ngcForces{};
		GspFeedback m_latestGspFeedback{};

		RelativeState m_latestRelativeState{};
		std::array<Pose, 2> m_latestPoses{};

		CsvLogger m_csvLogger{};
		TelemetryAccumulator m_accumulator{};
		uint64_t m_stepCount{0};
		double m_simTime{0.0};
	};
}
