/**
 * @file SpacecraftPoseComputation.cpp
 * @brief Implementation of platform pose computation from spacecraft states.
 */

#include "space_docking/dynamics_engine/SpacecraftPoseComputation.hpp"

namespace space_docking::dynamics
{
	std::array<Pose, 2> SpacecraftPoseComputation::compute(const BodyState& c,
	                                                       const BodyState& t,
	                                                       const GstFeedback& f) const noexcept
	{
		(void)f;
		return std::array<Pose, 2>{c.pose, t.pose};
	}
}
