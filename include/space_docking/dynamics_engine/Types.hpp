/**
 * @file Types.hpp
 * @brief Core mathematical, state representation, and telemetry types for the Space Docking Dynamics Engine.
 *
 * This header defines fundamental types including 3D vectors, unit quaternions q(w, x, y, z),
 * direction cosine matrices, unified 6-DOF spatial poses, rigid body states, and relative docking state vectors.
 *
 * Design Principle: Single set of canonical state variables.
 * To avoid data inconsistency across simulation modules, spatial position and attitude are encapsulated
 * exclusively within canonical Pose objects. Auxiliary parameters (such as Euler angles) are computed
 * on demand via member functions rather than maintained as separate state variables.
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace space_docking::dynamics
{
	// =========================================================================
	// Error Handling & Status Codes
	// =========================================================================
	/**
	 * @enum ErrorCode
	 * @brief Unified error and status codes for the entire simulation framework.
	 *
	 * Categorized into distinct numerical ranges:
	 *  -  0 to -9  : General engine status & lifecycle (OK, initialization, bad arguments)
	 *  - -10 to -19: Configuration & XML parsing (file missing, syntax errors, missing tags)
	 *  - -20 to -29: Spacecraft physical parameters (invalid mass, non-invertible inertia)
	 *  - -30 to -39: Mathematical / numerical integration (NaN/Inf divergence, singular matrices)
	 *  - -40 to -49: Flight safety & docking boundary violations (corridor breach, collision)
	 *  - -50 to -69: Timing deadlines, timeouts, and network/UDP communication errors
	 */
	enum class ErrorCode : int32_t
	{
		// General Status
		OK = 0,
		UNKNOWN_ERROR = -1,
		INVALID_ARGUMENT = -2,
		NOT_INITIALIZED = -3,

		// Configuration & XML (Cold Path)
		CONFIG_FILE_NOT_FOUND = -10,
		CONFIG_PARSE_ERROR = -11,
		CONFIG_MISSING_TAG = -12,
		CONFIG_INVALID_VALUE = -13,

		// Physical Spacecraft Parameters
		INVALID_MASS = -20,
		INVALID_INERTIA = -21,
		INVALID_TIMESTEP = -22,

		// Mathematical & Numerical Integration (Hot Path)
		STEP_FAILED = -30,
		MATH_NAN_OR_INF = -31,
		MATH_QUATERNION_ZERO_NORM = -32,
		MATH_SINGULAR_MATRIX = -33,

		// Flight Safety & Docking Corridor
		OUT_OF_BOUNDS = -40,
		DOCKING_COLLISION = -41,
		DOCKING_CORRIDOR_EXCEEDED = -42,
		PLATFORM_LIMIT_EXCEEDED = -43,

		// Real-time Timing & Communication
		TIMEOUT = -50,
		REALTIME_DEADLINE_MISSED = -51,
		NETWORK_ERROR = -60,
		SOCKET_BIND_FAILED = -61
	};

	/**
	 * @brief Converts an ErrorCode enum to a human-readable string literal.
	 * Evaluated at compile-time (constexpr) with zero runtime heap allocation.
	 */
	[[nodiscard]] constexpr const char* errorCodeToString(ErrorCode code) noexcept
	{
		switch (code)
		{
			case ErrorCode::OK: return "OK";
			case ErrorCode::UNKNOWN_ERROR: return "UNKNOWN_ERROR";
			case ErrorCode::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
			case ErrorCode::NOT_INITIALIZED: return "NOT_INITIALIZED";
			case ErrorCode::CONFIG_FILE_NOT_FOUND: return "CONFIG_FILE_NOT_FOUND";
			case ErrorCode::CONFIG_PARSE_ERROR: return "CONFIG_PARSE_ERROR";
			case ErrorCode::CONFIG_MISSING_TAG: return "CONFIG_MISSING_TAG";
			case ErrorCode::CONFIG_INVALID_VALUE: return "CONFIG_INVALID_VALUE";
			case ErrorCode::INVALID_MASS: return "INVALID_MASS";
			case ErrorCode::INVALID_INERTIA: return "INVALID_INERTIA";
			case ErrorCode::INVALID_TIMESTEP: return "INVALID_TIMESTEP";
			case ErrorCode::STEP_FAILED: return "STEP_FAILED";
			case ErrorCode::MATH_NAN_OR_INF: return "MATH_NAN_OR_INF";
			case ErrorCode::MATH_QUATERNION_ZERO_NORM: return "MATH_QUATERNION_ZERO_NORM";
			case ErrorCode::MATH_SINGULAR_MATRIX: return "MATH_SINGULAR_MATRIX";
			case ErrorCode::OUT_OF_BOUNDS: return "OUT_OF_BOUNDS";
			case ErrorCode::DOCKING_COLLISION: return "DOCKING_COLLISION";
			case ErrorCode::DOCKING_CORRIDOR_EXCEEDED: return "DOCKING_CORRIDOR_EXCEEDED";
			case ErrorCode::PLATFORM_LIMIT_EXCEEDED: return "PLATFORM_LIMIT_EXCEEDED";
			case ErrorCode::TIMEOUT: return "TIMEOUT";
			case ErrorCode::REALTIME_DEADLINE_MISSED: return "REALTIME_DEADLINE_MISSED";
			case ErrorCode::NETWORK_ERROR: return "NETWORK_ERROR";
			case ErrorCode::SOCKET_BIND_FAILED: return "SOCKET_BIND_FAILED";
		}
		return "UNKNOWN_ERROR_CODE";
	}

	enum class LogLevel : uint8_t
	{
		FATAL = 0,
		ERROR = 1,
		WARNING = 2
	};

	// =========================================================================
	// Coordinate Frame & Stewart Platform Configuration
	// =========================================================================
	enum class FrameMapping : uint8_t
	{
		DEFAULT = 0, // Chaser on Ground Stewart Platform (GSP), Target on Flight Stewart Platform (FSP)
		FLIPPED = 1  // Target on Ground Stewart Platform (GSP), Chaser on Flight Stewart Platform (FSP)
	};

	// =========================================================================
	// Fundamental Mathematical Types
	// =========================================================================

	/**
	 * @brief 3-dimensional vector in Cartesian space.
	 */
	struct Vector3
	{
		double x{0.0};
		double y{0.0};
		double z{0.0};

		constexpr Vector3() noexcept = default;
		constexpr Vector3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}
	};

	/**
	 * @brief Unit quaternion following standard q(w, x, y, z) notation.
	 * w is the scalar component; x, y, z are the vector components.
	 */
	struct Quaternion
	{
		double w{1.0}; // scalar component (rotation angle)
		double x{0.0}; // vector x component
		double y{0.0}; // vector y component
		double z{0.0}; // vector z component

		constexpr Quaternion() noexcept = default;
		constexpr Quaternion(double w_, double x_, double y_, double z_) noexcept
			: w(w_), x(x_), y(y_), z(z_) {}
	};

	/**
	 * @brief 3x3 Direction Cosine Matrix (Row-major layout).
	 */
	struct Matrix3
	{
		std::array<double, 9> m{
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0
		};

		constexpr Matrix3() noexcept = default;
		constexpr Matrix3(const std::array<double, 9>& elements) noexcept : m(elements) {}
	};

	/**
	 * @brief Intrinsic z-y-x (Yaw-Pitch-Roll) Euler angles representation in radians.
	 * Derived on demand to prevent state synchronization inconsistency.
	 */
	struct EulerAngles
	{
		double yaw{0.0};   // psi (rotation about z-axis)
		double pitch{0.0}; // theta (rotation about y-axis)
		double roll{0.0};  // phi (rotation about x-axis)
	};

	// =========================================================================
	// Canonical Spatial Pose & Force Representations
	// =========================================================================

	/**
	 * @brief Canonical 6-DOF spatial pose containing position and orientation quaternion q(w, x, y, z).
	 * Serves as the single source of truth for spatial location and attitude.
	 */
	struct Pose
	{
		Vector3 position{};
		Quaternion orientation{};

		constexpr Pose() noexcept = default;
		constexpr Pose(const Vector3& pos, const Quaternion& rot) noexcept
			: position(pos), orientation(rot) {}
	};

	/**
	 * @brief Combined 6-DOF Force and Torque structure (torque stored first).
	 */
	struct ForceTorque
	{
		Vector3 torque{};
		Vector3 force{};
	};

	// =========================================================================
	// Telemetry & Platform Feedback
	// =========================================================================

	/**
	 * @brief Live sensor telemetry from Ground Stewart Platform (GSP) and Flight Stewart Platform (FSP).
	 * Carries load-cell forces/torques and measured 6-DOF platform poses.
	 */
	struct Telemetry
	{
		double timestamp{0.0};
		ForceTorque groundStewartPlatformForces{}; // GSP load-cell forces/torques
		ForceTorque flightStewartPlatformForces{}; // FSP load-cell forces/torques
		Pose groundStewartPlatformPose{};          // GSP measured platform pose (position & orientation)
		Pose flightStewartPlatformPose{};          // FSP measured platform pose (position & orientation)
	};

	/**
	 * @brief Actuator feedback received from the Ground Stewart Platform (GSP).
	 */
	struct GspFeedback
	{
		std::array<double, 6> legLengths{};
		Pose pose{};
	};

	// Backwards compatibility alias
	using GstFeedback = GspFeedback;

	// =========================================================================
	// Spacecraft Dynamics State Vectors
	// =========================================================================

	/**
	 * @brief 6-DOF dynamic state of an individual spacecraft body in the inertial frame.
	 * Position and orientation are unified inside a single canonical Pose to prevent redundant state variables.
	 */
	struct BodyState
	{
		Pose pose{};           // Position (m) and orientation quaternion q(w, x, y, z)
		Vector3 velocity{};    // Linear velocity (m/s)
		Vector3 angularRate{}; // Angular rate (rad/s)
	};

	/**
	 * @brief 14-element relative docking motion state vector Y between Chaser and Target:
	 * Y = [position_rel, orientation_rel, velocity_rel, quaternionRate_rel].
	 * Spatial coordinates are unified within Pose; auxiliary Euler angles are derived on demand.
	 */
	struct RelativeState
	{
		Pose pose{};                 // Relative position and relative orientation quaternion q(w, x, y, z)
		Vector3 velocity{};          // Relative linear closing velocity (m/s)
		Quaternion quaternionRate{}; // Rate of change of relative quaternion qDot(w, x, y, z)

		/**
		 * @brief Converts orientation quaternion q(w, x, y, z) to intrinsic z-y-x Euler angles.
		 * Computed on-demand to guarantee state consistency without maintaining duplicate variables.
		 */
		[[nodiscard]] EulerAngles toEuler() const noexcept
		{
			EulerAngles euler;
			const double w = pose.orientation.w;
			const double x = pose.orientation.x;
			const double y = pose.orientation.y;
			const double z = pose.orientation.z;

			// Yaw (psi) about z-axis
			euler.yaw = std::atan2(2.0 * (w * z + x * y),
			                       1.0 - 2.0 * (y * y + z * z));
			// Pitch (theta) about y-axis
			const double sinp = 2.0 * (w * y - z * x);
			if (std::abs(sinp) >= 1.0)
			{
				euler.pitch = std::copysign(M_PI / 2.0, sinp); // clamp to +/- 90 degrees
			}
			else
			{
				euler.pitch = std::asin(sinp);
			}
			// Roll (phi) about x-axis
			euler.roll = std::atan2(2.0 * (w * x + y * z),
			                        1.0 - 2.0 * (x * x + y * y));

			return euler;
		}
	};

	// =========================================================================
	// Simulation Run Configuration & Visualization Snapshot
	// =========================================================================

	/**
	 * @brief Simulation run configuration parameters loaded dynamically from XML config.
	 */
	struct RunConfig
	{
		double chaserMass{0.0}; // Loaded dynamically from XML configuration
		double targetMass{0.0}; // Loaded dynamically from XML configuration
		Matrix3 chaserInertia{};
		Matrix3 targetInertia{};
		Vector3 initialOrbitElements{};
		Pose initialSeparation{};
		Vector3 chaserInitialVelocity{}; // Loaded dynamically from XML configuration
		Vector3 targetInitialVelocity{}; // Loaded dynamically from XML configuration
		FrameMapping frameMapping{FrameMapping::DEFAULT};

		// Logging configuration provision (loaded dynamically from XML)
		bool loggingEnabled{true};
		std::string logDirectory{"logs"};
		std::string logFileName{"simulation_trajectory.csv"};
		uint32_t logDecimation{10};
	};

	/**
	 * @brief State snapshot transmitted from Dynamics Engine to 3D Visualization / UI.
	 *
	 * Transmitted periodically (e.g. at 120 Hz over UDP) to drive real-time 3D rendering
	 * and force display gauges without duplicating simulation state variables.
	 */
	struct StateSnapshot
	{
		RelativeState relativeState{};
		ForceTorque chaserForces{}; // Applied/contact force and torque on Chaser
		ForceTorque targetForces{}; // Applied/contact force and torque on Target
	};
}
