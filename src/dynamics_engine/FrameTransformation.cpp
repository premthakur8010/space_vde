/**
 * @file FrameTransformation.cpp
 * @brief Implementation of coordinate frame transformations between platforms and spacecraft bodies.
 */

#include "space_docking/dynamics_engine/FrameTransformation.hpp"

namespace space_docking::dynamics
{
	namespace
	{
		// Multiplies 3x3 matrix by a 3D vector: v_out = M * v_in
		Vector3 multiplyMatrixVector(const Matrix3& m, const Vector3& v) noexcept
		{
			return Vector3{
				m.m[0] * v.x + m.m[1] * v.y + m.m[2] * v.z,
				m.m[3] * v.x + m.m[4] * v.y + m.m[5] * v.z,
				m.m[6] * v.x + m.m[7] * v.y + m.m[8] * v.z
			};
		}
	}

	void FrameTransformation::setFlipMode(FrameMapping mode) noexcept
	{
		m_flipMode = mode;
	}

	FrameMapping FrameTransformation::getFlipMode() const noexcept
	{
		return m_flipMode;
	}

	void FrameTransformation::setRotationPspBsp(const Matrix3& r) noexcept
	{
		m_R_Psp_Bsp = r;
	}

	void FrameTransformation::setRotationOspBsp(const Matrix3& r) noexcept
	{
		m_R_Osp_Bsp = r;
	}

	void FrameTransformation::setRotationDcBc(const Matrix3& r) noexcept
	{
		m_R_DC_BC = r;
	}

	void FrameTransformation::setRotationDtBt(const Matrix3& r) noexcept
	{
		m_R_DT_BT = r;
	}

	std::array<ForceTorque, 2> FrameTransformation::convertForces(const Telemetry& t) const
	{
		std::array<ForceTorque, 2> out{};

		if (m_flipMode == FrameMapping::DEFAULT)
		{
			// DEFAULT: Chaser on GSP (Ground Stewart Platform), Target on FSP (Flight Stewart Platform)
			out[0] = t.groundStewartPlatformForces;
			out[1] = t.flightStewartPlatformForces;
		}
		else
		{
			// FLIPPED: Target on GSP, Chaser on FSP
			out[0] = t.flightStewartPlatformForces;
			out[1] = t.groundStewartPlatformForces;
		}

		return out;
	}
}
