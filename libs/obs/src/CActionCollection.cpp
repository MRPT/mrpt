/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CActionCollection, CSerializable, mrpt::obs)

CActionCollection::CActionCollection(CAction& a) : m_actions()
{
	m_actions.emplace_back(CAction::Ptr(dynamic_cast<CAction*>(a.clone())));
}

uint8_t CActionCollection::serializeGetVersion() const { return 0; }
void CActionCollection::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(m_actions.size());
	for (const auto& a : *this) out << *a;
}

void CActionCollection::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			clear();
			m_actions.resize(in.ReadAs<uint32_t>());
			for_each(
				begin(), end(),
				mrpt::serialization::metaprogramming::
					ObjectReadFromStreamToPtrs<CAction::Ptr>(&in));
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
						clear
 ---------------------------------------------------------------*/
void CActionCollection::clear() { m_actions.clear(); }
/*---------------------------------------------------------------
						get
 ---------------------------------------------------------------*/
CAction::Ptr CActionCollection::get(size_t index)
{
	if (index >= m_actions.size()) THROW_EXCEPTION("Index out of bounds");

	return m_actions.at(index).get_ptr();
}

const CAction& CActionCollection::get(size_t index) const
{
	if (index >= m_actions.size()) THROW_EXCEPTION("Index out of bounds");

	return *(m_actions.at(index).get_ptr());
}

/*---------------------------------------------------------------
						size
 ---------------------------------------------------------------*/
size_t CActionCollection::size() { return m_actions.size(); }
/*---------------------------------------------------------------
						insert
 ---------------------------------------------------------------*/
void CActionCollection::insert(CAction& action)
{
	m_actions.emplace_back(
		CAction::Ptr(dynamic_cast<CAction*>(action.clone())));
}

/*---------------------------------------------------------------
						getBestMovementEstimation
 ---------------------------------------------------------------*/
CActionRobotMovement2D::Ptr CActionCollection::getBestMovementEstimation() const
{
	CActionRobotMovement2D::Ptr bestEst;
	double bestDet = 1e3;

	// Find the best
	for (const auto& it : *this)
	{
		if (it->GetRuntimeClass()->derivedFrom(
				CLASS_ID(CActionRobotMovement2D)))
		{
			CActionRobotMovement2D::Ptr temp =
				std::dynamic_pointer_cast<CActionRobotMovement2D>(it.get_ptr());

			if (temp->estimationMethod ==
				CActionRobotMovement2D::emScan2DMatching)
			{
				return temp;
			}

			double det = temp->poseChange->getCovariance().det();

			// If it is the best until now, save it:
			if (det < bestDet)
			{
				bestEst = temp;
				bestDet = det;
			}
		}
	}

	return bestEst;
}

/*---------------------------------------------------------------
							eraseByIndex
 ---------------------------------------------------------------*/
void CActionCollection::eraseByIndex(const size_t& index)
{
	if (index >= m_actions.size()) THROW_EXCEPTION("Index out of bounds");

	auto it = m_actions.begin() + index;
	m_actions.erase(it);
}

/*---------------------------------------------------------------
							eraseByIndex
 ---------------------------------------------------------------*/
CActionRobotMovement2D::Ptr CActionCollection::getMovementEstimationByType(
	CActionRobotMovement2D::TEstimationMethod method)
{
	// Find it:
	for (auto& it : *this)
	{
		if (it->GetRuntimeClass()->derivedFrom(
				CLASS_ID(CActionRobotMovement2D)))
		{
			CActionRobotMovement2D::Ptr temp =
				std::dynamic_pointer_cast<CActionRobotMovement2D>(it.get_ptr());

			// Is it of the required type?
			if (temp->estimationMethod == method)
			{
				// Yes!:
				return temp;
			}
		}
	}

	// Not found:
	return CActionRobotMovement2D::Ptr();
}

/*---------------------------------------------------------------
							erase
 ---------------------------------------------------------------*/
CActionCollection::iterator CActionCollection::erase(const iterator& it)
{
	MRPT_START
	ASSERT_(it != end());

	return m_actions.erase(it);
	MRPT_END
}

/*---------------------------------------------------------------
							getFirstMovementEstimationMean
 ---------------------------------------------------------------*/
bool CActionCollection::getFirstMovementEstimationMean(
	CPose3D& out_pose_increment) const
{
	CActionRobotMovement3D::Ptr act3D =
		getActionByClass<CActionRobotMovement3D>();
	if (act3D)
	{
		out_pose_increment = act3D->poseChange.mean;
		return true;
	}
	CActionRobotMovement2D::Ptr act2D =
		getActionByClass<CActionRobotMovement2D>();
	if (act2D)
	{
		out_pose_increment = CPose3D(act2D->poseChange->getMeanVal());
		return true;
	}
	return false;
}

/*---------------------------------------------------------------
					getFirstMovementEstimation
 ---------------------------------------------------------------*/
bool CActionCollection::getFirstMovementEstimation(
	CPose3DPDFGaussian& out_pose_increment) const
{
	CActionRobotMovement3D::Ptr act3D =
		getActionByClass<CActionRobotMovement3D>();
	if (act3D)
	{
		out_pose_increment = act3D->poseChange;
		return true;
	}
	CActionRobotMovement2D::Ptr act2D =
		getActionByClass<CActionRobotMovement2D>();
	if (act2D)
	{
		out_pose_increment.copyFrom(*act2D->poseChange);
		return true;
	}
	return false;
}
