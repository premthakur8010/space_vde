/**
 * @file SpacecraftPoseComputation.cpp
 * @brief Implementation of platform pose computation from spacecraft states and hardware feedback.
 *
 * This module supports Software-in-the-Loop (SIL) and Hardware-in-the-Loop (HIL) parity:
 * - On a developer laptop (SIL mode): No physical hexapod machine is connected. Feedback is 0.
 *   The function returns pure mathematical simulation poses.
 * - In the physical ISRO lab (HIL mode): Optical encoders on the 6 Stewart Platform legs
 *   stream the actual measured physical position. If physical tracking lag occurs, this
 *   module blends that physical offset with the simulated target pose.
 */

#include "space_docking/dynamics_engine/SpacecraftPoseComputation.hpp"

namespace space_docking::dynamics
{
	std::array<Pose, 2> SpacecraftPoseComputation::compute(const BodyState& c,
	                                                       const BodyState& t,
	                                                       const GspFeedback& f) const noexcept
	{
		// ---------------------------------------------------------------------
		// Hardware Presence Detection: Quaternion Norm Check
		// ---------------------------------------------------------------------
		// Any mathematically valid 3D orientation quaternion MUST be a unit quaternion:
		//   ||q||^2 = w^2 + x^2 + y^2 + z^2 == 1.0
		//
		// - If running on a laptop (no hardware connected):
		//   Feedback struct f is empty/zero-initialized -> qNormSq = 0.0 (< 0.1).
		// - If running with real Stewart Platform encoders:
		//   Valid orientation data streams in -> qNormSq ≈ 1.0 (> 0.1).
		const double qNormSq = f.pose.orientation.w * f.pose.orientation.w +
		                       f.pose.orientation.x * f.pose.orientation.x +
		                       f.pose.orientation.y * f.pose.orientation.y +
		                       f.pose.orientation.z * f.pose.orientation.z;

		if (qNormSq > 0.1)
		{
			// HARDWARE CONNECTED (HIL Mode):
			// Offset the target pose by the measured mechanical displacement of the platform
			Pose targetWithPlatform = t.pose;
			targetWithPlatform.position.x += f.pose.position.x;
			targetWithPlatform.position.y += f.pose.position.y;
			targetWithPlatform.position.z += f.pose.position.z;
			return std::array<Pose, 2>{c.pose, targetWithPlatform};
		}

		// SOFTWARE SIMULATION MODE (SIL Mode):
		// Directly return the pure simulated 3D poses for Chaser [0] and Target [1]
		return std::array<Pose, 2>{c.pose, t.pose};
	}
}
