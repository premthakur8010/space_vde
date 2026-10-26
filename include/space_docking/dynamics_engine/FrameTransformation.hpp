/**
 * @file FrameTransformation.hpp
 * @brief Converts sensor force and torque readings between motion platforms and spacecraft body frames.
 *
 * Handles both DEFAULT (Chaser on GST, Target on FST) and FLIPPED (Target on GST, Chaser on FST) mounting.
 */

#pragma once

#include "space_docking/dynamics_engine/Types.hpp"

namespace space_docking::dynamics
{
	/**
	 * @brief Converts sensor force/torque readings into each spacecraft's body frame
	 * using rotation matrices between platform, docking hardware, and spacecraft body frames.
	 */
	class FrameTransformation
	{
	public:
		FrameTransformation() = default;
		~FrameTransformation() = default;

		void setFlipMode(FrameMapping mode) noexcept;
		[[nodiscard]] FrameMapping getFlipMode() const noexcept;

		void setRotationPspBsp(const Matrix3& r) noexcept;
		void setRotationOspBsp(const Matrix3& r) noexcept;
		void setRotationDcBc(const Matrix3& r) noexcept;
		void setRotationDtBt(const Matrix3& r) noexcept;

		/**
		 * @brief Transform raw telemetry forces/torques from DAQ load cells into
		 * Chaser and Target body frames: [0] = Chaser, [1] = Target.
		 */
		[[nodiscard]] std::array<ForceTorque, 2> convertForces(const Telemetry& t) const;

	private:
		FrameMapping m_flipMode{FrameMapping::DEFAULT};
		Matrix3 m_R_Psp_Bsp{};
		Matrix3 m_R_Osp_Bsp{};
		Matrix3 m_R_DC_BC{};
		Matrix3 m_R_DT_BT{};
	};
}
