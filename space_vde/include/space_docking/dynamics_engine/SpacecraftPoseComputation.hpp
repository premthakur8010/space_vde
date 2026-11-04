/**
 * @file SpacecraftPoseComputation.hpp
 * @brief Computes motion platform pose commands from spacecraft body states and actuator feedback.
 */

#pragma once

#include "Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Computes Chaser and Target poses from body states and motion platform feedback.
	 *
	 * DUAL OPERATION MODES:
	 * 1. Software-in-the-Loop (SIL / Pure Simulation):
	 *    - When running without physical testbed hardware, the platform feedback quaternion
	 *      norm is zero (qNormSq <= 0.1).
	 *    - Pure simulated 6-DOF states of Chaser and Target are passed directly to output.
	 *
	 * 2. Hardware-in-the-Loop (HIL / Ground Stewart Platform Connected):
	 *    - When physical Stewart platform encoder feedback is active (qNormSq > 0.1),
	 *      the measured physical platform displacement and attitude offsets are blended
	 *      with the target pose:
	 *        r_target_blended = r_target_sim + r_platform_measured
	 *        q_target_blended = q_target_sim (x) q_platform_measured
	 *    - Compensates for physical actuator compliance and tracking lag during contact testing.
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
		                                          const GspFeedback& f) const noexcept;
	};
}
