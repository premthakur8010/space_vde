#pragma once

#include "space_docking/dynamics_engine/Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Abstract network boundary to external systems.
	 * Decouples physics and kinematics computation from socket transport mechanisms.
	 */
	class INetworkInterface
	{
	public:
		virtual ~INetworkInterface() = default;

		// Telemetry and feedback inputs
		virtual Telemetry getSensorForces() = 0;
		virtual std::array<ForceTorque, 2> getNgcForces() = 0;
		virtual GspFeedback getGspFeedback() = 0;
		virtual RunConfig getRunConfig() = 0;

		// Outputs
		virtual ErrorCode sendResults(const RelativeState& y, const BodyState& c, const BodyState& t) = 0;
		virtual ErrorCode sendState(const StateSnapshot& s) = 0;
	};
}
