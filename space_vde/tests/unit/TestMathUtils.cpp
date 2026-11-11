#include "space_docking/utils/MathUtils.hpp"

#include <cassert>
#include <iostream>

int main()
{
	using namespace space_docking::utils;

	// Test nearlyEqual with standard double values
	assert(nearlyEqual(1.0, 1.000000001, 1e-6));
	assert(!nearlyEqual(1.0, 1.01, 1e-4));

	// Test finite assertions on GLM vectors
	const glm::dvec3 validVec(1.0, -2.5, 42.0);
	assertFinite(validVec);

	// Test finite assertions on unit quaternions
	const glm::dquat validQuat(1.0, 0.0, 0.0, 0.0);
	assertFinite(validQuat);

	std::cout << "[PASS] TestMathUtils completed successfully.\n";
	return 0;
}
