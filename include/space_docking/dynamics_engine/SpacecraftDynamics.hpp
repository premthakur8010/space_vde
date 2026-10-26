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

#include "space_docking/dynamics_engine/Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Propagates 6-DOF rigid-body dynamics for a single spacecraft body.
	 * Instantiated once for the Chaser and once for the Target.
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
