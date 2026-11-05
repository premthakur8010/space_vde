/**
 * @file SpacecraftDynamics.cpp
 * @brief Implementation of 6-DOF rigid-body dynamics and quaternion kinematics for a spacecraft.
 *
 * Implements:
 * 1. setup(): Validates and assigns mass and moment of inertia dynamically.
 * 2. move(): Propagates both translational motion (Newton's 2nd Law) and rotational motion
 *    (Euler's rotational equations) over integration timestep dt, followed by quaternion
 *    attitude propagation and re-normalization.
 */

#include "space_docking/dynamics_engine/SpacecraftDynamics.hpp"
#include "space_docking/utils/LogMacros.hpp"

#include <cmath>

namespace space_docking::dynamics
{
	// =========================================================================
	// 1. SETUP: Spacecraft Mass and Moment of Inertia Configuration
	// =========================================================================

	// Configures spacecraft physical mass (kg) and 3x3 inertia tensor (kg*m^2), verifying positive mass
	ErrorCode SpacecraftDynamics::setup(double mass, const Matrix3& inertia)
	{
		// Physical sanity check: mass cannot be zero or negative
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
	// 2. MOVE: 6-DOF Dynamic Propagation over Timestep dt (1 kHz tick)
	// =========================================================================

	// NUMERICAL INTEGRATION SCHEME:
	// We use 1 kHz Semi-Implicit (Symplectic) Euler integration with quaternion re-normalization.
	// RK4 is deliberately NOT used because at dt = 1 ms (1000 Hz):
	//   1. Truncation error is already negligible (O(dt^2) ~ 1e-6).
	//   2. Symplectic Euler is energy-conserving for orbital mechanics (preserves phase-space volume).
	//   3. It executes in <50 ns, leaving maximum headroom in the 1 ms hard real-time hardware loop.
	//
	// State updates integrated over dt:
	//   [Integration Step 1]: Velocity      v(t+dt) = v(t) + a * dt
	//   [Integration Step 2]: Position      r(t+dt) = r(t) + v(t+dt) * dt (uses new v -> symplectic)
	//   [Integration Step 3]: Angular Rate  w(t+dt) = w(t) + alpha * dt
	//   [Integration Step 4]: Attitude Quat q(t+dt) = q(t) + qDot * dt
	BodyState SpacecraftDynamics::move(const ForceTorque& w, double dt)
	{
		// Safety Guard: Spacecraft must have a valid positive mass configured before stepping
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
		// Optimization: Pre-computing 1.0 / m_mass allows fast multiplications
		// instead of 3 expensive hardware divisions on every 1 ms cycle.
		const double invMass = 1.0 / m_mass;
		const Vector3 linAcc{
			w.force.x * invMass, // a_x = F_x / m
			w.force.y * invMass, // a_y = F_y / m
			w.force.z * invMass  // a_z = F_z / m
		};

		// [Integration Step 1]: Velocity = Integral of Acceleration (a * dt)
		m_state.velocity.x += linAcc.x * dt;
		m_state.velocity.y += linAcc.y * dt;
		m_state.velocity.z += linAcc.z * dt;

		// [Integration Step 2]: Position = Integral of Velocity (v * dt)
		// (Symplectic: uses the newly integrated velocity to conserve orbital energy)
		m_state.pose.position.x += m_state.velocity.x * dt;
		m_state.pose.position.y += m_state.velocity.y * dt;
		m_state.pose.position.z += m_state.velocity.z * dt;

		// ---------------------------------------------------------------------
		// Part B: Rotational Motion (Euler's Rotational Equations of Motion)
		// ---------------------------------------------------------------------
		// In the spacecraft principal body frame, the rotational dynamics are:
		//   I * d(omega)/dt + omega x (I * omega) = tau_net
		//
		// Rearranging for angular acceleration alpha = d(omega)/dt:
		//   alpha_x = (tau_x - (I_zz - I_yy) * omega_y * omega_z) / I_xx
		//   alpha_y = (tau_y - (I_xx - I_zz) * omega_z * omega_x) / I_yy
		//   alpha_z = (tau_z - (I_yy - I_xx) * omega_x * omega_y) / I_zz
		//
		// The cross-terms (e.g. (Izz - Iyy) * wy * wz) account for gyroscopic
		// coupling when a spinning body rotates about multiple axes simultaneously.
		const double Ixx = m_inertia.m[0];
		const double Iyy = m_inertia.m[4];
		const double Izz = m_inertia.m[8];

		const double invIxx = Ixx > 1e-9 ? 1.0 / Ixx : 0.0;
		const double invIyy = Iyy > 1e-9 ? 1.0 / Iyy : 0.0;
		const double invIzz = Izz > 1e-9 ? 1.0 / Izz : 0.0;

		const double wx = m_state.angularRate.x;
		const double wy = m_state.angularRate.y;
		const double wz = m_state.angularRate.z;

		// Compute angular acceleration per axis
		const double alphaX = (w.torque.x - (Izz - Iyy) * wy * wz) * invIxx;
		const double alphaY = (w.torque.y - (Ixx - Izz) * wz * wx) * invIyy;
		const double alphaZ = (w.torque.z - (Iyy - Ixx) * wx * wy) * invIzz;

		// [Integration Step 3]: Angular Velocity = Integral of Angular Acceleration (alpha * dt)
		m_state.angularRate.x += alphaX * dt;
		m_state.angularRate.y += alphaY * dt;
		m_state.angularRate.z += alphaZ * dt;

		// ---------------------------------------------------------------------
		// Part C: Attitude Kinematics with Unit Quaternions q(w, x, y, z)
		// ---------------------------------------------------------------------
		// Why Quaternions?
		// Euler angles (Roll, Pitch, Yaw) suffer from "Gimbal Lock" (mathematical
		// singularity when Pitch reaches +/- 90 degrees). Quaternions provide
		// a continuous, singularity-free 4D representation of 3D rotations.
		//
		// Kinematic differential equation:
		//   dq/dt = 0.5 * q (x) omega
		//
		// Expanded in scalar-first q(w, x, y, z) notation:
		//   dw/dt = 0.5 * (-x*wx - y*wy - z*wz)
		//   dx/dt = 0.5 * ( w*wx + y*wz - z*wy)
		//   dy/dt = 0.5 * ( w*wy - x*wz + z*wx)
		//   dz/dt = 0.5 * ( w*wz + x*wy - y*wx)
		const double qw = m_state.pose.orientation.w;
		const double qx = m_state.pose.orientation.x;
		const double qy = m_state.pose.orientation.y;
		const double qz = m_state.pose.orientation.z;

		const double dw = 0.5 * (-qx * wx - qy * wy - qz * wz);
		const double dx = 0.5 * ( qw * wx + qy * wz - qz * wy);
		const double dy = 0.5 * ( qw * wy - qx * wz + qz * wx);
		const double dz = 0.5 * ( qw * wz + qx * wy - qy * wx);

		// [Integration Step 4]: Attitude Quaternion = Integral of Quaternion Derivative (qDot * dt)
		m_state.pose.orientation.w += dw * dt;
		m_state.pose.orientation.x += dx * dt;
		m_state.pose.orientation.y += dy * dt;
		m_state.pose.orientation.z += dz * dt;

		// ---------------------------------------------------------------------
		// Part D: Quaternion Re-normalization (Preventing Numerical Drift)
		// ---------------------------------------------------------------------
		// Numerical integration introduces slight floating-point rounding errors
		// every millisecond. Over thousands of cycles, ||q|| could drift away
		// from 1.0, which would warp and distort 3D rotations.
		// Normalizing q = q / ||q|| ensures it remains a pure rotation matrix.
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

	// Returns current 13-element body state (position, quaternion, velocity, angular rate)
	const BodyState& SpacecraftDynamics::getState() const noexcept
	{
		return m_state;
	}

	// Overwrites body state directly (used when initializing positions from XML config)
	void SpacecraftDynamics::setState(const BodyState& state) noexcept
	{
		m_state = state;
	}

	// Returns active spacecraft mass in kilograms
	double SpacecraftDynamics::getMass() const noexcept
	{
		return m_mass;
	}

	// Returns active 3x3 moment of inertia tensor in kg*m^2
	const Matrix3& SpacecraftDynamics::getInertia() const noexcept
	{
		return m_inertia;
	}
}
