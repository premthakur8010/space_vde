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
#include <numbers>
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
		SOCKET_BIND_FAILED = -61,

		// Logging & File I/O Status Codes
		FILE_OPEN_FAILED = -70,  ///< Could not create or open destination file on disk
		LOG_WRITE_FAILED = -71   ///< Disk write error or I/O failure during telemetry recording
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
			case ErrorCode::FILE_OPEN_FAILED: return "FILE_OPEN_FAILED";
			case ErrorCode::LOG_WRITE_FAILED: return "LOG_WRITE_FAILED";
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
	enum class FrameMapping : int32_t
	{
		DEFAULT = 0, // Chaser on Ground Stewart Platform (GSP), Target on Flight Stewart Platform (FSP)
		FLIPPED = 1, // Target on Ground Stewart Platform (GSP), Chaser on Flight Stewart Platform (FSP)
		Default = 0, // UI alias
		Flipped = 1  // UI alias
	};

	// =========================================================================
	// Fundamental Mathematical Types
	// =========================================================================

	/**
	 * @brief 3-dimensional Cartesian vector: [x, y, z]^T in SI units.
	 *
	 * Used across the dynamics engine to represent:
	 * - Positions [m] (e.g. center of mass location, docking port coordinates)
	 * - Linear velocities [m/s]
	 * - Angular velocities / rates [rad/s]
	 * - Linear forces [N]
	 * - Angular torques [N*m]
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
	 * @brief Unit quaternion for 3D attitude representation following standard q(w, x, y, z) notation.
	 *
	 * MATHEMATICAL FORMULATION:
	 * A rotation of angle theta about an arbitrary unit axis u = [u_x, u_y, u_z]^T is represented as:
	 *   q = [w, x, y, z]^T
	 * where:
	 *   w = cos(theta / 2)           (scalar component)
	 *   x = u_x * sin(theta / 2)     (vector component along X)
	 *   y = u_y * sin(theta / 2)     (vector component along Y)
	 *   z = u_z * sin(theta / 2)     (vector component along Z)
	 *
	 * PROPERTIES & ADVANTAGES:
	 * 1. Unit Norm: Must satisfy ||q|| = sqrt(w^2 + x^2 + y^2 + z^2) = 1.0.
	 * 2. Singularity-Free: Unlike Euler angles, quaternions have NO gimbal lock at +/- 90 deg pitch.
	 * 3. Identity Rotation: q = [1.0, 0.0, 0.0, 0.0]^T represents zero rotation.
	 * 4. Inverse / Conjugate: For unit quaternions, q^-1 = q* = [w, -x, -y, -z]^T.
	 */
	struct Quaternion
	{
		double w{1.0}; // Scalar component (rotation angle magnitude cos(theta/2))
		double x{0.0}; // Vector component along X axis
		double y{0.0}; // Vector component along Y axis
		double z{0.0}; // Vector component along Z axis

		constexpr Quaternion() noexcept = default;
		constexpr Quaternion(double w_, double x_, double y_, double z_) noexcept
			: w(w_), x(x_), y(y_), z(z_) {}
	};

	/**
	 * @brief 3x3 Matrix stored in Row-Major order.
	 *
	 * Used for:
	 * 1. Direction Cosine Matrices (DCM / Rotation matrices R in SO(3)):
	 *    Transforms vectors between reference frames: v_inertial = R * v_body.
	 * 2. Spacecraft Mass Moments of Inertia Tensor J [kg*m^2]:
	 *    J = [ Ixx  -Ixy  -Ixz ]
	 *        [ -Iyx  Iyy  -Iyz ]
	 *        [ -Izx  -Izy  Izz ]
	 *    Stored sequentially: m[0]=Ixx, m[1]=Ixy, m[2]=Ixz, m[3]=Iyx, etc.
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
	 * @brief Intrinsic Z-Y-X (Yaw-Pitch-Roll) Tait-Bryan Euler angles in radians.
	 *
	 * ORDER OF ROTATIONS:
	 * 1. Yaw   (psi)   : Rotation about body Z axis [-pi, +pi]
	 * 2. Pitch (theta) : Rotation about intermediate Y axis [-pi/2, +pi/2]
	 * 3. Roll  (phi)   : Rotation about final X axis [-pi, +pi]
	 *
	 * ARCHITECTURAL RULE:
	 * Euler angles are derived ON DEMAND via toEuler() or quatToEuler() for human display
	 * and GUI gauges. They are NEVER stored as continuous simulation state variables
	 * to prevent Gimbal Lock singularities (division by zero at pitch = +/- 90 degrees).
	 */
	struct EulerAngles
	{
		double roll{0.0};  // phi (rotation about x-axis, rad)
		double pitch{0.0}; // theta (rotation about y-axis, rad)
		double yaw{0.0};   // psi (rotation about z-axis, rad)
	};
	using Rpy = EulerAngles;
	using Vec3 = Vector3;
	using Mat3 = Matrix3;

	// =========================================================================
	// Canonical Spatial Pose & Force Representations
	// =========================================================================

	/**
	 * @brief Canonical 6-DOF spatial pose: position [m] + orientation quaternion q(w, x, y, z).
	 *
	 * Serves as the single unified source of truth for 3D spatial location and attitude.
	 * By coupling position and orientation into a single structure, we ensure that
	 * translation and rotation are always updated atomically without synchronization skew.
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
	 * @brief Combined 6-DOF Force and Torque wrench structure.
	 *
	 * NOTE ON FIELD ORDERING:
	 * Torque is stored FIRST, followed by Force second, matching the hardware interface
	 * specification and OpenGL telemetry layout of the VDE testbed.
	 */
	struct ForceTorque
	{
		Vector3 torque{}; // [N*m] Torques about X, Y, Z axes
		Vector3 force{};  // [N]   Forces along X, Y, Z axes
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
	 * Contains the 6 individual actuator strut lengths [m] and the measured 6-DOF platform pose.
	 */
	struct GspFeedback
	{
		std::array<double, 6> legLengths{}; // Individual strut actuator stroke lengths [m]
		Pose pose{};                        // 6-DOF platform pose calculated from forward kinematics
	};

	// Backwards compatibility alias
	using GstFeedback = GspFeedback;

	// =========================================================================
	/**
	 * @brief Kinematics and contact/actuator loads of the physical VDE Stewart platform and overhead gantry.
	 * Telemetry consumed by the 3D visualization dashboard to render physical testbed motions.
	 */
	struct VdeState
	{
		Vector3 positionM{};              // Platform position in facility frame [m]
		Vector3 linearRateMS{};            // Linear platform velocity [m/s]
		EulerAngles orientationRad{};       // Platform roll, pitch, yaw [rad]
		EulerAngles orientationRateRadS{};  // Platform angular rates [rad/s]
		ForceTorque moving{};              // Moving hexapod load-cell wrench (torque, then force)
		ForceTorque overhead{};            // Overhead gantry load-cell wrench (torque, then force)
	};

	/**
	 * @brief Mass properties, body-frame rates, and Earth-Centred Inertial (ECI) orbital state of a spacecraft.
	 */
	struct SpacecraftState
	{
		double massKg{0.0};                 // Spacecraft dry + propellant mass [kg]
		Vector3 cgM{};                      // Center of Gravity offset [m]
		Matrix3 inertiaKgM2{};              // 3x3 moment of inertia tensor [kg*m^2]
		Vector3 linearRateMS{};             // Body-fixed linear velocity [m/s]
		EulerAngles orientationRateRadS{};   // Body-fixed angular rates [rad/s]
		Vector3 eciPositionM{};             // Inertial position vector in ECI frame [m]
		Vector3 eciVelocityMS{};            // Inertial velocity vector in ECI frame [m/s]
		EulerAngles eciOrientationRad{};    // Spacecraft attitude relative to ECI frame [rad]
		EulerAngles eciOrientationRateRadS{};// Spacecraft attitude rates relative to ECI [rad/s]
	};

	// =========================================================================
	// Spacecraft Dynamics & Relative Motion State Vectors
	// =========================================================================

	/**
	 * @brief 13-element canonical dynamic state vector X for an individual spacecraft body in the inertial frame:
	 *   X = [ r_x, r_y, r_z,   (position, 3 elements)
	 *         q_w, q_x, q_y, q_z, (quaternion attitude, 4 elements)
	 *         v_x, v_y, v_z,   (linear velocity, 3 elements)
	 *         w_x, w_y, w_z ]  (angular rate, 3 elements)
	 * Total: 13 scalar state variables.
	 *
	 * Position and orientation are unified inside a single canonical Pose to prevent redundant state variables.
	 */
	struct BodyState
	{
		Pose pose{};           // Position [m] and unit attitude quaternion q(w, x, y, z)
		Vector3 velocity{};    // Linear velocity [m/s] in inertial frame
		Vector3 angularRate{}; // Angular rate [rad/s] in spacecraft body frame
	};

	/**
	 * @brief 14-element relative docking motion state vector Y between Chaser and Target:
	 *   Y = [ r_rel,        (relative position vector, 3 elements)
	 *         q_rel,        (relative attitude quaternion, 4 elements)
	 *         v_rel,        (relative linear velocity, 3 elements)
	 *         qDot_rel ]    (rate of change of relative quaternion, 4 elements)
	 * Total: 14 scalar state variables.
	 *
	 * DEFINITIONS:
	 * 1. Relative Position: r_rel = r_chaser - r_target [m]
	 * 2. Relative Velocity: v_rel = v_chaser - v_target [m/s]
	 * 3. Relative Attitude: q_rel = q_target^-1 (x) q_chaser (rotation from Target to Chaser frame)
	 * 4. Relative Quaternion Derivative: qDot_rel = 0.5 * q_rel (x) [0, w_rel]^T
	 */
	struct RelativeState
	{
		Pose pose{};                 // Relative position and relative orientation quaternion q(w, x, y, z)
		Vector3 velocity{};          // Relative linear closing velocity (m/s)
		Quaternion quaternionRate{}; // Rate of change of relative quaternion qDot(w, x, y, z)

		/**
		 * @brief Converts orientation quaternion q(w, x, y, z) to intrinsic Z-Y-X Euler angles (radians).
		 *
		 * MATHEMATICAL FORMULATION:
		 * From the Direction Cosine Matrix R(q):
		 *   R_11 = 1 - 2(y^2 + z^2),   R_12 = 2(xy - wz),        R_13 = 2(xz + wy)
		 *   R_21 = 2(xy + wz),        R_22 = 1 - 2(x^2 + z^2),   R_23 = 2(yz - wx)
		 *   R_31 = 2(xz - wy),        R_32 = 2(yz + wx),        R_33 = 1 - 2(x^2 + y^2)
		 *
		 * Inverse extraction:
		 * 1. Yaw   (psi)   = atan2(R_21, R_11) = atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
		 * 2. Pitch (theta) = -asin(R_31)       = asin(2(w*y - z*x))
		 *    - Clamped to +/- pi/2 to prevent NaN when numerical roundoff causes |sinp| > 1.0 (Gimbal Lock)
		 * 3. Roll  (phi)   = atan2(R_32, R_33) = atan2(2(w*x + y*z), 1 - 2(x^2 + y^2))
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
				// Clamp to +/- 90 degrees if outside domain due to numerical floating-point inaccuracies
				euler.pitch = std::copysign(std::numbers::pi / 2.0, sinp);
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
