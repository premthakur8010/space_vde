/**
 * @file DynamicsEngine.hpp
 * @brief Central physics simulation coordinator for the Space Docking System.
 *
 * Coordinates execution of Chaser and Target 6-DOF dynamics, frame transformations,
 * relative docking kinematics, and platform pose computations at 1 kHz cycle frequency.
 */

#pragma once

#include "space_docking/dynamics_engine/FrameTransformation.hpp"
#include "space_docking/dynamics_engine/RelativeKinematics.hpp"
#include "space_docking/dynamics_engine/SpacecraftDynamics.hpp"
#include "space_docking/dynamics_engine/SpacecraftPoseComputation.hpp"
#include "space_docking/dynamics_engine/Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Central physics coordinator for the Dynamics Engine.
	 * Executes on a dedicated cyclic thread.
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
		 * @brief Execute one physics cycle step.
		 * @param dt Step size in seconds (e.g. 0.001 s for 1 kHz).
		 */
		ErrorCode step(double dt);

		void setTelemetry(const Telemetry& t) noexcept;
		void setNgcForces(const std::array<ForceTorque, 2>& f) noexcept;
		void setGstFeedback(const GstFeedback& f) noexcept;

		[[nodiscard]] const SpacecraftDynamics& getChaser() const noexcept;
		[[nodiscard]] const SpacecraftDynamics& getTarget() const noexcept;
		[[nodiscard]] const RelativeState& getRelativeState() const noexcept;
		[[nodiscard]] const std::array<Pose, 2>& getPoses() const noexcept;

	private:
		RunConfig m_config{};
		bool m_initialized{false};

		SpacecraftDynamics m_chaser{};
		SpacecraftDynamics m_target{};

		FrameTransformation m_frameTransform{};
		RelativeKinematics m_relativeKinematics{};
		SpacecraftPoseComputation m_poseComputation{};

		Telemetry m_currentTelemetry{};
		std::array<ForceTorque, 2> m_ngcForces{};
		GstFeedback m_latestGstFeedback{};

		RelativeState m_latestRelativeState{};
		std::array<Pose, 2> m_latestPoses{};
	};
}
