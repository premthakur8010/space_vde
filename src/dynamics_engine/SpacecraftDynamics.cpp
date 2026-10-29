/**
 * @file SpacecraftDynamics.cpp
 * @brief Implementation of 6-DOF rigid-body dynamics and quaternion kinematics for a spacecraft.
 *
 * Implements:
 * 1. setup(): Validates and assigns mass and moment of inertia dynamically.
 * 2. move(): Translates body using Newton's Second Law (a = F/m), computes angular accelerations
 *    via Euler's rotational equations (alpha = I^-1 * (tau - omega x (I * omega))), and integrates
 *    attitude using standard unit quaternion kinematics q(w, x, y, z).
 */

#include "space_docking/dynamics_engine/SpacecraftDynamics.hpp"
#include "space_docking/utils/LogMacros.hpp"

#include <cmath>

namespace space_docking::dynamics
{
	// =========================================================================
	// 1. SETUP: Spacecraft Mass and Moment of Inertia Configuration
	// =========================================================================
	ErrorCode SpacecraftDynamics::setup(double mass, const Matrix3& inertia)
	{
		if (mass <= 0.0)
		{
			SD_LOG_ERROR(ErrorCode::INVALID_MASS, "SpacecraftDynamics::setup() rejected non-positive mass: " << mass << " kg (must be > 0.0)");
			return ErrorCode::INVALID_MASS;
		}

		m_mass = mass;
		m_inertia = inertia;

		return ErrorCode::OK;
	}

	// =========================================================================
	// 2. MOVE: 6-DOF Dynamic Propagation over Timestep dt
	// =========================================================================
	BodyState SpacecraftDynamics::move(const ForceTorque& w, double dt)
	{
		// Guard: Spacecraft must have a valid mass configured dynamically via setup()
		if (m_mass <= 0.0)
		{
			SD_LOG_ERROR(ErrorCode::NOT_INITIALIZED, "SpacecraftDynamics::move() called on uninitialized body (mass is 0 or unconfigured)");
			return m_state;
		}

		if (dt <= 0.0)
		{
			SD_LOG_ERROR(ErrorCode::INVALID_TIMESTEP, "SpacecraftDynamics::move() called with non-positive timestep dt: " << dt);
			return m_state;
		}

		// ---------------------------------------------------------------------
		// Part A: Translational Motion (Newton's Second Law: a = F / m)
		// ---------------------------------------------------------------------
		const double invMass = 1.0 / m_mass;
		const Vector3 linAcc{
			w.force.x * invMass,
			w.force.y * invMass,
			w.force.z * invMass
		};

		// Update linear velocity
		m_state.velocity.x += linAcc.x * dt;
		m_state.velocity.y += linAcc.y * dt;
		m_state.velocity.z += linAcc.z * dt;

		// Update position in canonical Pose
		m_state.pose.position.x += m_state.velocity.x * dt;
		m_state.pose.position.y += m_state.velocity.y * dt;
		m_state.pose.position.z += m_state.velocity.z * dt;

		// ---------------------------------------------------------------------
		// Part B: Rotational Motion (Euler's Rotational Equations)
		// ---------------------------------------------------------------------
		const double Ixx = m_inertia.m[0];
		const double Iyy = m_inertia.m[4];
		const double Izz = m_inertia.m[8];

		const double invIxx = Ixx > 1e-9 ? 1.0 / Ixx : 0.0;
		const double invIyy = Iyy > 1e-9 ? 1.0 / Iyy : 0.0;
		const double invIzz = Izz > 1e-9 ? 1.0 / Izz : 0.0;

		const double wx = m_state.angularRate.x;
		const double wy = m_state.angularRate.y;
		const double wz = m_state.angularRate.z;

		const double alphaX = (w.torque.x - (Izz - Iyy) * wy * wz) * invIxx;
		const double alphaY = (w.torque.y - (Ixx - Izz) * wz * wx) * invIyy;
		const double alphaZ = (w.torque.z - (Iyy - Ixx) * wx * wy) * invIzz;

		m_state.angularRate.x += alphaX * dt;
		m_state.angularRate.y += alphaY * dt;
		m_state.angularRate.z += alphaZ * dt;

		// ---------------------------------------------------------------------
		// Part C: Attitude Kinematics with Standard Quaternion q(w, x, y, z)
		// ---------------------------------------------------------------------
		const double qw = m_state.pose.orientation.w;
		const double qx = m_state.pose.orientation.x;
		const double qy = m_state.pose.orientation.y;
		const double qz = m_state.pose.orientation.z;

		// Quaternion kinematic derivative: dq/dt = 0.5 * q * omega
		const double dw = 0.5 * (-qx * wx - qy * wy - qz * wz);
		const double dx = 0.5 * ( qw * wx + qy * wz - qz * wy);
		const double dy = 0.5 * ( qw * wy - qx * wz + qz * wx);
		const double dz = 0.5 * ( qw * wz + qx * wy - qy * wx);

		m_state.pose.orientation.w += dw * dt;
		m_state.pose.orientation.x += dx * dt;
		m_state.pose.orientation.y += dy * dt;
		m_state.pose.orientation.z += dz * dt;

		// Renormalize quaternion to prevent numerical drift
		const double norm = std::sqrt(m_state.pose.orientation.w * m_state.pose.orientation.w +
		                              m_state.pose.orientation.x * m_state.pose.orientation.x +
		                              m_state.pose.orientation.y * m_state.pose.orientation.y +
		                              m_state.pose.orientation.z * m_state.pose.orientation.z);
		if (norm > 1e-12)
		{
			const double invNorm = 1.0 / norm;
			m_state.pose.orientation.w *= invNorm;
			m_state.pose.orientation.x *= invNorm;
			m_state.pose.orientation.y *= invNorm;
			m_state.pose.orientation.z *= invNorm;
		}

		return m_state;
	}

	// =========================================================================
	// 3. State Accessors & Mutators
	// =========================================================================
	const BodyState& SpacecraftDynamics::getState() const noexcept
	{
		return m_state;
	}

	void SpacecraftDynamics::setState(const BodyState& state) noexcept
	{
		m_state = state;
	}

	double SpacecraftDynamics::getMass() const noexcept
	{
		return m_mass;
	}

	const Matrix3& SpacecraftDynamics::getInertia() const noexcept
	{
		return m_inertia;
	}
}
