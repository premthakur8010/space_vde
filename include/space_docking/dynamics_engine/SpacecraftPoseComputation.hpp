/**
 * @file SpacecraftPoseComputation.hpp
 * @brief Computes motion platform pose commands from spacecraft body states and actuator feedback.
 */

#pragma once

#include "space_docking/dynamics_engine/Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Computes Chaser and Target poses from body states and motion platform feedback.
	 */
	class SpacecraftPoseComputation
	{
	public:
		SpacecraftPoseComputation() = default;
		~SpacecraftPoseComputation() = default;

		/**
		 * @brief Compute Chaser and Target poses.
		 * @param c Chaser body state.
		 * @param t Target body state.
		 * @param f Feedback received from motion platform.
		 * @return std::array<Pose, 2>: [0] = Chaser pose, [1] = Target pose.
		 */
		[[nodiscard]] std::array<Pose, 2> compute(const BodyState& c,
		                                          const BodyState& t,
		                                          const GstFeedback& f) const noexcept;
	};
}
