/**
 * @file RelativeKinematics.cpp
 * @brief Implementation of relative docking kinematics and quaternion transformations.
 *
 * Computes:
 * 1. Relative position: r_rel = r_chaser - r_target
 * 2. Relative linear velocity: v_rel = v_chaser - v_target
 * 3. Relative orientation quaternion: q_rel = q_target^-1 * q_chaser using standard q(w, x, y, z)
 * 4. Quaternion rate of change: qDot_rel = 0.5 * q_rel * (omega_chaser - omega_target)
 */

#include "space_docking/dynamics_engine/RelativeKinematics.hpp"

namespace space_docking::dynamics
{
	namespace
	{
		/**
		 * @brief Quaternion multiplication following standard q(w, x, y, z) notation.
		 */
		constexpr Quaternion multiplyQuaternions(const Quaternion& a, const Quaternion& b) noexcept
		{
			return Quaternion{
				a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, // w
				a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, // x
				a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, // y
				a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w  // z
			};
		}
	}

	RelativeState RelativeKinematics::compute(const BodyState& chaser, const BodyState& target) const noexcept
	{
		RelativeState rel;

		// Relative position: r_rel = r_chaser - r_target
		rel.pose.position.x = chaser.pose.position.x - target.pose.position.x;
		rel.pose.position.y = chaser.pose.position.y - target.pose.position.y;
		rel.pose.position.z = chaser.pose.position.z - target.pose.position.z;

		// Relative linear velocity: v_rel = v_chaser - v_target
		rel.velocity.x = chaser.velocity.x - target.velocity.x;
		rel.velocity.y = chaser.velocity.y - target.velocity.y;
		rel.velocity.z = chaser.velocity.z - target.velocity.z;

		// Relative orientation quaternion: q_rel = q_target^-1 * q_chaser
		// For unit quaternion q(w, x, y, z), inverse conjugate is q(w, -x, -y, -z)
		const Quaternion targetInv{
			target.pose.orientation.w,
			-target.pose.orientation.x,
			-target.pose.orientation.y,
			-target.pose.orientation.z
		};
		rel.pose.orientation = multiplyQuaternions(targetInv, chaser.pose.orientation);

		// Relative angular rate: omega_rel = omega_chaser - omega_target
		const Vector3 dOmega{
			chaser.angularRate.x - target.angularRate.x,
			chaser.angularRate.y - target.angularRate.y,
			chaser.angularRate.z - target.angularRate.z
		};

		// Quaternion kinematic derivative: qDot = 0.5 * q_rel * omega_rel
		const double qw = rel.pose.orientation.w;
		const double qx = rel.pose.orientation.x;
		const double qy = rel.pose.orientation.y;
		const double qz = rel.pose.orientation.z;

		rel.quaternionRate.w = 0.5 * (-qx * dOmega.x - qy * dOmega.y - qz * dOmega.z);
		rel.quaternionRate.x = 0.5 * ( qw * dOmega.x + qy * dOmega.z - qz * dOmega.y);
		rel.quaternionRate.y = 0.5 * ( qw * dOmega.y - qx * dOmega.z + qz * dOmega.x);
		rel.quaternionRate.z = 0.5 * ( qw * dOmega.z + qx * dOmega.y - qy * dOmega.x);

		return rel;
	}
}
