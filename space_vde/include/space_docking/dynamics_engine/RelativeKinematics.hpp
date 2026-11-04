/**
 * @file RelativeKinematics.hpp
 * @brief Computes the 14-element relative docking motion state vector Y between Chaser and Target spacecraft.
 *
 * This class calculates the relative spatial pose (position and orientation quaternion q(w, x, y, z)),
 * relative linear velocity, and the relative quaternion derivative between Chaser and Target.
 * State consistency is enforced by housing relative position and orientation within a single Pose struct.
 */

#pragma once

#include "Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Computes relative docking state vector Y between Chaser and Target.
	 *
	 * MATHEMATICAL FOUNDATION:
	 * 1. Relative Position Vector:
	 *    r_rel = r_chaser - r_target [m]
	 *
	 * 2. Relative Linear Velocity Vector:
	 *    v_rel = v_chaser - v_target [m/s]
	 *
	 * 3. Relative Attitude Quaternion:
	 *    q_rel = q_target^-1 (x) q_chaser
	 *    Transforms vectors from the Target body coordinate system to the Chaser body coordinate system.
	 *
	 * 4. Relative Angular Velocity:
	 *    omega_rel = omega_chaser - R(q_rel) * omega_target [rad/s]
	 *
	 * 5. Relative Quaternion Derivative:
	 *    qDot_rel = 0.5 * q_rel (x) [0, omega_rel]^T
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
