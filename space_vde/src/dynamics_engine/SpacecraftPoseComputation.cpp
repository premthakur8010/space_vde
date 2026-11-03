/**
 * @file SpacecraftPoseComputation.cpp
 * @brief Implementation of platform pose computation from spacecraft states.
 */

#include "space_docking/dynamics_engine/SpacecraftPoseComputation.hpp"

namespace space_docking::dynamics
{
	std::array<Pose, 2> SpacecraftPoseComputation::compute(const BodyState& c,
	                                                       const BodyState& t,
	                                                       const GspFeedback& f) const noexcept
	{
		// If physical Stewart platform feedback is present (valid orientation norm),
		// integrate the measured platform pose offset with the target pose.
		const double qNormSq = f.pose.orientation.w * f.pose.orientation.w +
		                       f.pose.orientation.x * f.pose.orientation.x +
		                       f.pose.orientation.y * f.pose.orientation.y +
		                       f.pose.orientation.z * f.pose.orientation.z;

		if (qNormSq > 0.1)
		{
			Pose targetWithPlatform = t.pose;
			targetWithPlatform.position.x += f.pose.position.x;
			targetWithPlatform.position.y += f.pose.position.y;
			targetWithPlatform.position.z += f.pose.position.z;
			return std::array<Pose, 2>{c.pose, targetWithPlatform};
		}

		return std::array<Pose, 2>{c.pose, t.pose};
	}
}
