/**
 * @file RelativeKinematics.cpp
 * @brief Implementation of relative docking kinematics and quaternion transformations.
 *
 * During spacecraft docking, cameras, LiDAR, and docking sensors do not care about Earth-Centered
 * Inertial (ECI) coordinates. They exclusively measure the relative separation, approach speed,
 * and attitude misalignment between the Chaser probe and Target cone.
 *
 * This class calculates:
 * 1. Relative position vector:    r_rel = r_chaser - r_target
 * 2. Relative velocity vector:    v_rel = v_chaser - v_target
 * 3. Relative orientation quat:   q_rel = q_target^-1 (x) q_chaser
 * 4. Relative quaternion rate:    qDot_rel = 0.5 * q_rel (x) (omega_chaser - omega_target)
 */

#include "space_docking/dynamics_engine/RelativeKinematics.hpp"

namespace space_docking::dynamics
{
	namespace
	{
		/**
		 * @brief Quaternion multiplication (Hamilton product) following scalar-first q(w, x, y, z) notation.
		 *
		 * Given quaternions a = (w_a, v_a) and b = (w_b, v_b):
		 *   w_out = w_a*w_b - v_a . v_b
		 *   v_out = w_a*v_b + w_b*v_a + v_a x v_b
		 */
		constexpr Quaternion multiplyQuaternions(const Quaternion& a, const Quaternion& b) noexcept
		{
			return Quaternion{
				a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, // w (scalar product - dot product)
				a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, // x (vector cross-product component)
				a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, // y (vector cross-product component)
				a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w  // z (vector cross-product component)
			};
		}
	}

	RelativeState RelativeKinematics::compute(const BodyState& chaser, const BodyState& target) const noexcept
	{
		RelativeState rel;

		// ---------------------------------------------------------------------
		// 1. Relative Position: Distance from Target to Chaser
		// ---------------------------------------------------------------------
		// r_rel = r_chaser - r_target
		// If Chaser is at x = -10 m (along docking approach axis) and Target is at x = 0 m:
		// r_rel.x = -10.0 m (10 meters closing distance).
		rel.pose.position.x = chaser.pose.position.x - target.pose.position.x;
		rel.pose.position.y = chaser.pose.position.y - target.pose.position.y;
		rel.pose.position.z = chaser.pose.position.z - target.pose.position.z;

		// ---------------------------------------------------------------------
		// 2. Relative Linear Velocity: Closing Speed
		// ---------------------------------------------------------------------
		// v_rel = v_chaser - v_target
		// A positive v_rel.x means the gap is closing towards the target cone.
		rel.velocity.x = chaser.velocity.x - target.velocity.x;
		rel.velocity.y = chaser.velocity.y - target.velocity.y;
		rel.velocity.z = chaser.velocity.z - target.velocity.z;

		// ---------------------------------------------------------------------
		// 3. Relative Orientation Quaternion: q_rel = q_target^-1 (x) q_chaser
		// ---------------------------------------------------------------------
		// Why multiply inverse target by chaser?
		// In rotation algebra, to find the rotation needed to align the Target
		// frame with the Chaser frame:
		//   R_rel = (R_target)^T * R_chaser
		//
		// For a unit quaternion q = [w, x, y, z], the mathematical inverse is
		// simply its conjugate: q^* = [w, -x, -y, -z].
		const Quaternion targetInv{
			target.pose.orientation.w,
			-target.pose.orientation.x,
			-target.pose.orientation.y,
			-target.pose.orientation.z
		};
		rel.pose.orientation = multiplyQuaternions(targetInv, chaser.pose.orientation);

		// ---------------------------------------------------------------------
		// 4. Relative Angular Rate: Differential Spin
		// ---------------------------------------------------------------------
		// omega_rel = omega_chaser - omega_target
		const Vector3 dOmega{
			chaser.angularRate.x - target.angularRate.x,
			chaser.angularRate.y - target.angularRate.y,
			chaser.angularRate.z - target.angularRate.z
		};

		// ---------------------------------------------------------------------
		// 5. Relative Quaternion Derivative: dq_rel / dt
		// ---------------------------------------------------------------------
		// Formula: qDot_rel = 0.5 * q_rel (x) omega_rel
		// This tells navigation filters (e.g. Extended Kalman Filters) how fast
		// the angular misalignment is rotating per second.
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
