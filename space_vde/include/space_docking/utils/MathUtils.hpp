#pragma once

#include <cmath>
#include <numbers>
#include <cassert>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../dynamics_engine/Types.hpp"

/**
 * @brief Mathematical utilities and numerical tolerances for physical simulation.
 * Units: Standard SI (meters, radians, seconds).
 * Thread-safety: Pure functions; inherently thread-safe.
 */
namespace space_docking::utils
{
	constexpr double kDefaultTolerance = 1e-6;
	constexpr double kQuaternionTolerance = 1e-9;

	/// Millimetre <-> meter conversion (config files use mm; engine state uses SI meters).
	constexpr double kMmToM = 1e-3;
	constexpr double kMToMm = 1e3;

	/// OpenGL world units per millimetre. 1.0 => 1:1 mapping (1 GL unit = 1 mm).
	constexpr double kOpenGlUnitsPerMm = 1.0;

	/**
	 * @brief Convert a position in millimetres (target reference frame) to OpenGL world units.
	 * Axes are identical (+X docking axis, +Y up, +Z right), so only scaling is applied.
	 */
	[[nodiscard]] constexpr double mmToOpenGl(double mm) noexcept
	{
		return mm * kOpenGlUnitsPerMm;
	}

	/**
	 * @brief Convert an engine position in meters to OpenGL world units (via millimetres).
	 */
	[[nodiscard]] constexpr double metersToOpenGl(double meters) noexcept
	{
		return mmToOpenGl(meters * kMToMm);
	}

	/**
	 * @brief Floating-point comparison with an explicit epsilon.
	 * @param a First value.
	 * @param b Second value.
	 * @param epsilon Tolerance margin (meaning: absolute threshold for near-zero, relative otherwise).
	 * @return true if within tolerance, false otherwise.
	 */
	[[nodiscard]] constexpr bool nearlyEqual(double a, double b, double epsilon = kDefaultTolerance) noexcept
	{
		const double diff = a > b ? a - b : b - a;
		return diff <= epsilon;
	}

	[[nodiscard]] constexpr bool nearlyEqual(float a, float b, float epsilon = static_cast<float>(kDefaultTolerance)) noexcept
	{
		const float diff = a > b ? a - b : b - a;
		return diff <= epsilon;
	}

	/**
	 * @brief Assert that a vector contains only finite real numbers (no NaN or Inf).
	 */
	inline void assertFinite(const glm::dvec3& vec) noexcept
	{
		assert(std::isfinite(vec.x) && "Non-finite X component detected");
		assert(std::isfinite(vec.y) && "Non-finite Y component detected");
		assert(std::isfinite(vec.z) && "Non-finite Z component detected");
	}

	/**
	 * @brief Assert that a quaternion contains only finite real numbers.
	 */
	inline void assertFinite(const glm::dquat& quat) noexcept
	{
		assert(std::isfinite(quat.w) && "Non-finite W component detected");
		assert(std::isfinite(quat.x) && "Non-finite X component detected");
		assert(std::isfinite(quat.y) && "Non-finite Y component detected");
		assert(std::isfinite(quat.z) && "Non-finite Z component detected");
	}

	/**
	 * @brief Converts a unit quaternion q(w, x, y, z) into intrinsic Z-Y-X Roll-Pitch-Yaw Euler angles (radians).
	 *
	 * ROTATION CONVENTION:
	 * Intrinsic Z-Y-X Tait-Bryan angles:
	 * 1. Yaw   (psi)   around body Z axis: atan2( 2(w*z + x*y), 1 - 2(y^2 + z^2) )
	 * 2. Pitch (theta) around intermediate Y axis: asin( 2(w*y - z*x) )
	 *    - Clamped to +/- pi/2 to prevent NaN if numerical roundoff pushes |sinp| > 1.0 (Gimbal Lock)
	 * 3. Roll  (phi)   around final X axis: atan2( 2(w*x + y*z), 1 - 2(x^2 + y^2) )
	 */
	[[nodiscard]] inline space_docking::dynamics::EulerAngles quatToEuler(const space_docking::dynamics::Quaternion& q) noexcept
	{
		space_docking::dynamics::EulerAngles euler{};
		const double w = q.w, x = q.x, y = q.y, z = q.z;
		euler.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
		const double sinp = 2.0 * (w * y - z * x);
		euler.pitch = (std::abs(sinp) >= 1.0) ? std::copysign(std::numbers::pi / 2.0, sinp) : std::asin(sinp);
		euler.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
		return euler;
	}

	/**
	 * @brief Converts intrinsic Z-Y-X Roll-Pitch-Yaw Euler angles (radians) into a unit quaternion q(w, x, y, z).
	 *
	 * MATHEMATICAL FORMULATION:
	 * Half-angle trigonometric decomposition:
	 *   cr = cos(roll / 2),  sr = sin(roll / 2)
	 *   cp = cos(pitch / 2), sp = sin(pitch / 2)
	 *   cy = cos(yaw / 2),   sy = sin(yaw / 2)
	 *
	 * Quaternion components via Hamilton product q = q_yaw * q_pitch * q_roll:
	 *   w = cr*cp*cy + sr*sp*sy
	 *   x = sr*cp*cy - cr*sp*sy
	 *   y = cr*sp*cy + sr*cp*sy
	 *   z = cr*cp*sy - sr*sp*cy
	 */
	[[nodiscard]] inline space_docking::dynamics::Quaternion eulerToQuat(const space_docking::dynamics::EulerAngles& r) noexcept
	{
		const double cr = std::cos(r.roll * 0.5),  sr = std::sin(r.roll * 0.5);
		const double cp = std::cos(r.pitch * 0.5), sp = std::sin(r.pitch * 0.5);
		const double cy = std::cos(r.yaw * 0.5),   sy = std::sin(r.yaw * 0.5);
		return space_docking::dynamics::Quaternion{
			cr * cp * cy + sr * sp * sy, // w (scalar)
			sr * cp * cy - cr * sp * sy, // x (vector)
			cr * sp * cy + sr * cp * sy, // y (vector)
			cr * cp * sy - sr * sp * cy  // z (vector)
		};
	}
}
