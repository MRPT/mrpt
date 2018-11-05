/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::io;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::serialization;

IMPLEMENTS_SERIALIZABLE(CRawlog, CSerializable, mrpt::obs)

// ctor
CRawlog::CRawlog() : m_seqOfActObs(), m_commentTexts() {}
// dtor
CRawlog::~CRawlog() { clear(); }
void CRawlog::clear()
{
	m_seqOfActObs.clear();
	m_commentTexts.text.clear();
}

void CRawlog::addObservations(CSensoryFrame& observations)
{
	m_seqOfActObs.push_back(std::dynamic_pointer_cast<CSerializable>(
		observations.duplicateGetSmartPtr()));
}

void CRawlog::addActions(CActionCollection& actions)
{
	m_seqOfActObs.push_back(std::dynamic_pointer_cast<CSerializable>(
		actions.duplicateGetSmartPtr()));
}

void CRawlog::addActionsMemoryReference(const CActionCollection::Ptr& action)
{
	m_seqOfActObs.push_back(action);
}

void CRawlog::addObservationsMemoryReference(
	const CSensoryFrame::Ptr& observations)
{
	m_seqOfActObs.push_back(observations);
}
void CRawlog::addGenericObject(const CSerializable::Ptr& obj)
{
	m_seqOfActObs.push_back(obj);
}

void CRawlog::addObservationMemoryReference(
	const CObservation::Ptr& observation)
{
	if (IS_CLASS(observation, CObservationComment))
	{
		CObservationComment::Ptr o =
			std::dynamic_pointer_cast<CObservationComment>(observation);
		m_commentTexts = *o;
	}
	else
		m_seqOfActObs.push_back(observation);
}

void CRawlog::addAction(CAction& action)
{
	CActionCollection::Ptr temp =
		mrpt::make_aligned_shared<CActionCollection>();
	temp->insert(action);
	m_seqOfActObs.push_back(temp);
}

size_t CRawlog::size() const { return m_seqOfActObs.size(); }
CActionCollection::Ptr CRawlog::getAsAction(size_t index) const
{
	MRPT_START

	if (index >= m_seqOfActObs.size()) THROW_EXCEPTION("Index out of bounds");

	CSerializable::Ptr obj = m_seqOfActObs[index];

	if (obj->GetRuntimeClass() == CLASS_ID(CActionCollection))
		return std::dynamic_pointer_cast<CActionCollection>(obj);
	else
		THROW_EXCEPTION_FMT(
			"Element at index %i is not a CActionCollection", (int)index);
	MRPT_END
}

CObservation::Ptr CRawlog::getAsObservation(size_t index) const
{
	MRPT_START

	if (index >= m_seqOfActObs.size()) THROW_EXCEPTION("Index out of bounds");

	CSerializable::Ptr obj = m_seqOfActObs[index];

	if (obj->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
		return std::dynamic_pointer_cast<CObservation>(obj);
	else
		THROW_EXCEPTION_FMT(
			"Element at index %i is not a CObservation", (int)index);
	MRPT_END
}

CSerializable::Ptr CRawlog::getAsGeneric(size_t index) const
{
	MRPT_START
	if (index >= m_seqOfActObs.size()) THROW_EXCEPTION("Index out of bounds");

	return m_seqOfActObs[index];
	MRPT_END
}

CRawlog::TEntryType CRawlog::getType(size_t index) const
{
	MRPT_START
	if (index >= m_seqOfActObs.size()) THROW_EXCEPTION("Index out of bounds");

	const CSerializable::Ptr& obj = m_seqOfActObs[index];

	if (obj->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
		return etObservation;
	else if (obj->GetRuntimeClass() == CLASS_ID(CActionCollection))
		return etActionCollection;
	else if (obj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
		return etSensoryFrame;
	else
		return etOther;

	MRPT_END
}

CSensoryFrame::Ptr CRawlog::getAsObservations(size_t index) const
{
	MRPT_START
	if (index >= m_seqOfActObs.size()) THROW_EXCEPTION("Index out of bounds");

	CSerializable::Ptr obj = m_seqOfActObs[index];

	if (obj->GetRuntimeClass()->derivedFrom(CLASS_ID(CSensoryFrame)))
		return std::dynamic_pointer_cast<CSensoryFrame>(obj);
	else
		THROW_EXCEPTION_FMT(
			"Element at index %i is not a CSensoryFrame", (int)index);
	MRPT_END
}

uint8_t CRawlog::serializeGetVersion() const { return 1; }
void CRawlog::serializeTo(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<uint32_t>(m_seqOfActObs.size());
	for (const auto& a : m_seqOfActObs) out << a;
	out << m_commentTexts;
}

void CRawlog::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			clear();
			m_seqOfActObs.resize(in.ReadAs<uint32_t>());
			for (auto& a : m_seqOfActObs) a = in.ReadObject();
			in >> m_commentTexts;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

bool CRawlog::loadFromRawLogFile(
	const std::string& fileName, bool non_obs_objects_are_legal)
{
	// Open for read.
	CFileGZInputStream fi(fileName);
	if (!fi.fileOpenCorrectly()) return false;
	auto fs = archiveFrom(fi);

	clear();  // Clear first

	// OK: read objects:
	bool keepReading = true;
	while (keepReading)
	{
		CSerializable::Ptr newObj;
		try
		{
			fs >> newObj;
			bool add_obj = false;
			// Check type:
			if (newObj->GetRuntimeClass() == CLASS_ID(CRawlog))
			{
				// It is an entire object: Copy and finish:
				CRawlog::Ptr ao = std::dynamic_pointer_cast<CRawlog>(newObj);
				this->swap(*ao);
				return true;
			}
			else if (newObj->GetRuntimeClass()->derivedFrom(
						 CLASS_ID(CObservation)))
			{
				if (IS_CLASS(newObj, CObservationComment))
				{
					CObservationComment::Ptr o =
						std::dynamic_pointer_cast<CObservationComment>(newObj);
					m_commentTexts = *o;
				}
				else
				{
					add_obj = true;
				}
			}
			else if (newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
			{
				add_obj = true;
			}
			else if (newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
			{
				add_obj = true;
			}
			else
			{
				// Other classes:
				if (non_obs_objects_are_legal)
				{
					add_obj = true;
				}
				else
				{
					keepReading = false;
				}
			}
			if (add_obj) m_seqOfActObs.push_back(newObj);
		}
		catch (CExceptionEOF&)
		{  // EOF, just finish the loop
			keepReading = false;
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			keepReading = false;
		}
		catch (...)
		{
			keepReading = false;
		}
	}
	return true;
}

void CRawlog::remove(size_t index)
{
	MRPT_START
	if (index >= m_seqOfActObs.size()) THROW_EXCEPTION("Index out of bounds");
	m_seqOfActObs.erase(m_seqOfActObs.begin() + index);
	MRPT_END
}

void CRawlog::remove(size_t first_index, size_t last_index)
{
	MRPT_START
	if (first_index >= m_seqOfActObs.size() ||
		last_index >= m_seqOfActObs.size())
		THROW_EXCEPTION("Index out of bounds");
	m_seqOfActObs.erase(
		m_seqOfActObs.begin() + first_index,
		m_seqOfActObs.begin() + last_index + 1);
	MRPT_END
}

bool CRawlog::saveToRawLogFile(const std::string& fileName) const
{
	try
	{
		CFileGZOutputStream fo(fileName);
		auto f = archiveFrom(fo);
		if (!m_commentTexts.text.empty()) f << m_commentTexts;
		for (const auto& m_seqOfActOb : m_seqOfActObs) f << *m_seqOfActOb;
		return true;
	}
	catch (...)
	{
		return false;
	}
}

void CRawlog::swap(CRawlog& obj)
{
	if (this == &obj) return;
	m_seqOfActObs.swap(obj.m_seqOfActObs);
	std::swap(m_commentTexts, obj.m_commentTexts);
}

bool CRawlog::readActionObservationPair(
	CArchive& inStream, CActionCollection::Ptr& action,
	CSensoryFrame::Ptr& observations, size_t& rawlogEntry)
{
	try
	{
		// Load pose change from the rawlog:
		action.reset();
		while (!action)
		{
			CSerializable::Ptr obj;
			inStream >> obj;
			if (obj->GetRuntimeClass() == CLASS_ID(CActionCollection))
			{
				action = std::dynamic_pointer_cast<CActionCollection>(obj);
			}
			else
			{
				obj.reset();
			}
			rawlogEntry++;
		};

		// Load sensory frame from the rawlog:
		observations.reset();
		while (!observations)
		{
			CSerializable::Ptr obj;
			inStream >> obj;
			if (obj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
			{
				observations = std::dynamic_pointer_cast<CSensoryFrame>(obj);
			}
			else
			{
				obj.reset();
			}
			rawlogEntry++;
		}
		return true;
	}
	catch (CExceptionEOF&)
	{
		return false;
	}
	catch (const std::exception& e)
	{
		std::cerr << "[CRawlog::readActionObservationPair] Found exception:"
				  << std::endl
				  << e.what() << std::endl;
		return false;
	}
	catch (...)
	{
		std::cerr << "Untyped exception reading rawlog file!!" << std::endl;
		return false;
	}
}

bool CRawlog::getActionObservationPairOrObservation(
	CArchive& inStream, CActionCollection::Ptr& action,
	CSensoryFrame::Ptr& observations, CObservation::Ptr& observation,
	size_t& rawlogEntry)
{
	try
	{
		// Load pose change from the rawlog:
		observations.reset();
		observation.reset();
		action.reset();
		while (!action)
		{
			CSerializable::Ptr obj;
			inStream >> obj;
			if (IS_CLASS(obj, CActionCollection))
			{
				action = std::dynamic_pointer_cast<CActionCollection>(obj);
			}
			else if (IS_DERIVED(obj, CObservation))
			{
				observation = std::dynamic_pointer_cast<CObservation>(obj);
				rawlogEntry++;
				return true;
			}
			else
				obj.reset();
			rawlogEntry++;
		};

		// Load sensory frame from the rawlog:
		observations.reset();
		while (!observations)
		{
			CSerializable::Ptr obj;
			inStream >> obj;
			if (obj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
			{
				observations = std::dynamic_pointer_cast<CSensoryFrame>(obj);
			}
			rawlogEntry++;
		}
		return true;
	}
	catch (CExceptionEOF&)
	{
		return false;
	}
	catch (const std::exception& e)
	{
		std::cerr << "[CRawlog::readActionObservationPair] Found exception:"
				  << std::endl
				  << e.what() << std::endl;
		return false;
	}
	catch (...)
	{
		std::cerr << "Untyped exception reading rawlog file!!" << std::endl;
		return false;
	}
}

void CRawlog::findObservationsByClassInRange(
	mrpt::system::TTimeStamp time_start, mrpt::system::TTimeStamp time_end,
	const mrpt::rtti::TRuntimeClassId* class_type,
	TListTimeAndObservations& out_found, size_t guess_start_position) const
{
	MRPT_UNUSED_PARAM(guess_start_position);
	MRPT_START

	out_found.clear();

	if (m_seqOfActObs.empty()) return;

	// Find the first appearance of time_start:
	// ---------------------------------------------------
	auto first = m_seqOfActObs.begin();
	const auto last = m_seqOfActObs.end();
	{
		// The following is based on lower_bound:
		size_t count, step;
		count = std::distance(first, last);
		while (count > 0)
		{
			auto it = first;
			step = count / 2;
			std::advance(it, step);

			// The comparison function:
			TTimeStamp this_timestamp;
			if ((*it)->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
			{
				CObservation::Ptr o =
					std::dynamic_pointer_cast<CObservation>(*it);
				this_timestamp = o->timestamp;
				ASSERT_(this_timestamp != INVALID_TIMESTAMP);
			}
			else
				THROW_EXCEPTION(
					"Element found which is not derived from CObservation");

			if (this_timestamp < time_start)  // *it < time_start
			{
				first = ++it;
				count -= step + 1;
			}
			else
				count = step;
		}
		// "first" is our guy
	}

	// Iterate until we get out of the time window:
	while (first != last)
	{
		TTimeStamp this_timestamp;
		if ((*first)->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
		{
			CObservation::Ptr o =
				std::dynamic_pointer_cast<CObservation>(*first);
			this_timestamp = o->timestamp;
			ASSERT_(this_timestamp != INVALID_TIMESTAMP);

			if (this_timestamp < time_end)
			{
				if (o->GetRuntimeClass()->derivedFrom(class_type))
					out_found.insert(TTimeObservationPair(this_timestamp, o));
			}
			else
			{
				break;  // end of time window!
			}
		}
		else
			THROW_EXCEPTION(
				"Element found which is not derived from CObservation");

		first++;
	}

	MRPT_END
}

bool CRawlog::getActionObservationPair(
	CActionCollection::Ptr& action, CSensoryFrame::Ptr& observations,
	size_t& rawlogEntry) const
{
	try
	{
		while (getType(rawlogEntry) != CRawlog::etActionCollection)
		{
			rawlogEntry++;
		}
		action = getAsAction(rawlogEntry++);

		while (getType(rawlogEntry) != CRawlog::etSensoryFrame)
		{
			rawlogEntry++;
		}
		observations = getAsObservations(rawlogEntry++);

		return true;
	}
	catch (std::exception&)
	{
		return false;
	}
	catch (...)
	{
		std::cerr << "Untyped exception getting act-obs pair from rawlog!!"
				  << std::endl;
		return false;
	}
}

void CRawlog::getCommentText(std::string& t) const { t = m_commentTexts.text; }
std::string CRawlog::getCommentText() const { return m_commentTexts.text; }
void CRawlog::getCommentTextAsConfigFile(
	mrpt::config::CConfigFileMemory& memCfg) const
{
	memCfg.setContent(m_commentTexts.text);
}

void CRawlog::setCommentText(const std::string& t) { m_commentTexts.text = t; }
std::string CRawlog::detectImagesDirectory(const std::string& str)
{
	const std::string rawlog_path = extractFileDirectory(str);
	std::string temptative_img_path =
		rawlog_path + extractFileName(str) + std::string("_Images");
	if (mrpt::system::fileExists(temptative_img_path))
		return temptative_img_path;
	else if (mrpt::system::fileExists(
				 temptative_img_path =
					 (rawlog_path + extractFileName(str) +
					  std::string("_images"))))
		return temptative_img_path;
	else if (mrpt::system::fileExists(
				 temptative_img_path =
					 (rawlog_path + extractFileName(str) +
					  std::string("_IMAGES"))))
		return temptative_img_path;
	else
		return rawlog_path + "Images";
}
