/**
 * @file RelativeKinematics.hpp
 * @brief Computes the 14-element relative docking motion state vector Y between Chaser and Target spacecraft.
 *
 * This class calculates the relative spatial pose (position and orientation quaternion q(w, x, y, z)),
 * relative linear velocity, and the relative quaternion derivative between Chaser and Target.
 * State consistency is enforced by housing relative position and orientation within a single Pose struct.
 */

#pragma once

#include "space_docking/dynamics_engine/Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Computes relative docking state vector Y between Chaser and Target.
	 */
	class RelativeKinematics
	{
	public:
		RelativeKinematics() = default;
		~RelativeKinematics() = default;

		/**
		 * @brief Compute relative docking state between Chaser and Target.
		 * @param chaser Chaser BodyState in inertial frame.
		 * @param target Target BodyState in inertial frame.
		 * @return RelativeState containing relative pose, velocity, and quaternion derivative.
		 */
		[[nodiscard]] RelativeState compute(const BodyState& chaser, const BodyState& target) const noexcept;
	};
}
