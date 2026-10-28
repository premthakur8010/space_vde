#pragma once

#include <cmath>
#include <cassert>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

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
}
