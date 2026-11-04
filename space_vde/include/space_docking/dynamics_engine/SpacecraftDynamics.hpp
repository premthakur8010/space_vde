/**
 * @file SpacecraftDynamics.hpp
 * @brief 6-DOF rigid-body dynamics propagator for an individual spacecraft body (Chaser or Target).
 *
 * Propagates translational motion using Newton's Second Law and rotational motion using
 * Euler's equations of motion. Spacecraft attitude is integrated using standard unit quaternion
 * kinematics q(w, x, y, z).
 *
 * Spacecraft physical parameters (mass and inertia tensor) are initialized dynamically via setup().
 * Single canonical state is maintained in BodyState.
 */

#pragma once

#include "Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Propagates 6-DOF rigid-body dynamics for a single spacecraft body.
	 *
	 * INSTANTIATION:
	 * Instantiated twice in the Dynamics Engine:
	 * 1. m_chaser: The active chaser spacecraft performing the docking maneuver.
	 * 2. m_target: The target spacecraft (station-keeping or tumbling in orbit).
	 *
	 * PHYSICAL GOVERNING EQUATIONS:
	 * 1. Translational Dynamics (Newton's 2nd Law in Inertial Frame):
	 *    a = F_total / m
	 *    v(t + dt) = v(t) + a * dt
	 *    r(t + dt) = r(t) + v(t) * dt + 0.5 * a * dt^2
	 *
	 * 2. Rotational Dynamics (Euler's Equations of Motion in Body Frame):
	 *    J * d(omega)/dt + omega x (J * omega) = tau_body
	 *    d(omega)/dt = J^-1 * [ tau_body - omega x (J * omega) ]
	 *    where J is the 3x3 mass moment of inertia tensor, and omega is angular velocity.
	 *    The term "omega x (J * omega)" represents non-linear gyroscopic coupling
	 *    (e.g., (Izz - Iyy)*wy*wz).
	 *
	 * 3. Attitude Kinematics (Unit Quaternion Integration):
	 *    dq/dt = 0.5 * q (x) [0, omega]^T
	 *    where (x) is the Hamilton quaternion cross-product.
	 *    q(t + dt) is re-normalized to ||q|| = 1.0 to prevent numerical drift.
	 */
	class SpacecraftDynamics
	{
	public:
		SpacecraftDynamics() = default;
		~SpacecraftDynamics() = default;

		/**
		 * @brief Configure spacecraft mass and moment of inertia tensor dynamically.
		 * @param mass Mass in kilograms (must be greater than 0.0).
		 * @param inertia 3x3 inertia matrix in body-fixed coordinates.
		 * @return ErrorCode::OK if parameters are valid, ErrorCode::INVALID_ARGUMENT otherwise.
		 */
		ErrorCode setup(double mass, const Matrix3& inertia);

		/**
		 * @brief Propagate equations of motion given applied 6-DOF forces and torques over dt.
		 * @param w Applied force and torque (torque first, then force).
		 * @param dt Integration step size in seconds.
		 * @return Updated canonical BodyState containing pose, velocity, and angular rate.
		 */
		BodyState move(const ForceTorque& w, double dt);

		[[nodiscard]] const BodyState& getState() const noexcept;
		void setState(const BodyState& state) noexcept;

		[[nodiscard]] double getMass() const noexcept;
		[[nodiscard]] const Matrix3& getInertia() const noexcept;

	private:
		// Default initialized to 0.0 kg; must be configured dynamically via setup() during run initialization
		double m_mass{0.0};
		Matrix3 m_inertia{};
		BodyState m_state{};
	};
}
