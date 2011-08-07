/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>   // Precompiled headers

#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CRawlog, CSerializable,mrpt::slam)

/*---------------------------------------------------------------
					Default constructor
  ---------------------------------------------------------------*/
CRawlog::CRawlog() : m_seqOfActObs(), m_commentTexts()
{
}

/*---------------------------------------------------------------
					Destructor
  ---------------------------------------------------------------*/
CRawlog::~CRawlog()
{
	clear();
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CRawlog::clear()
{
	// Since we use smart pointers, there's no need to manually delete object...
	// for_each(m_seqOfActObs.begin(), m_seqOfActObs.end(),  ObjectDelete() );
	m_seqOfActObs.clear();

	m_commentTexts.text.clear();
}

/*---------------------------------------------------------------
						clearWithoutDelete
  ---------------------------------------------------------------*/
void  CRawlog::clearWithoutDelete()
{
	m_seqOfActObs.clear();
	m_commentTexts.text.clear();
}


/*---------------------------------------------------------------
						addObservation
  ---------------------------------------------------------------*/
void  CRawlog::addObservations(
		CSensoryFrame		&observations )
{
	m_seqOfActObs.push_back( CSerializablePtr( observations.duplicateGetSmartPtr() ) );
}

/*---------------------------------------------------------------
					addActions
  ---------------------------------------------------------------*/
void  CRawlog::addActions(
		CActionCollection		&actions )
{
	m_seqOfActObs.push_back( CSerializablePtr( actions.duplicateGetSmartPtr() ) );
}

/*---------------------------------------------------------------
					addActionsMemoryReference
  ---------------------------------------------------------------*/
void  CRawlog::addActionsMemoryReference( const CActionCollectionPtr &action )
{
	m_seqOfActObs.push_back( action );
}

/*---------------------------------------------------------------
					addObservationsMemoryReference
  ---------------------------------------------------------------*/
void  CRawlog::addObservationsMemoryReference( const CSensoryFramePtr &observations )
{
	m_seqOfActObs.push_back( observations );
}

/*---------------------------------------------------------------
					addObservationMemoryReference
  ---------------------------------------------------------------*/
void  CRawlog::addObservationMemoryReference( const CObservationPtr &observation )
{
	if (IS_CLASS(observation,CObservationComment))
	{
		CObservationCommentPtr o = CObservationCommentPtr(observation);
		m_commentTexts = *o;
	}
	else
	m_seqOfActObs.push_back( observation );
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void  CRawlog::addAction( CAction &action )
{
	CActionCollection	*temp = new CActionCollection();
	temp->insert( action );
	m_seqOfActObs.push_back( CSerializablePtr(temp) );
}

/*---------------------------------------------------------------
						size
  ---------------------------------------------------------------*/
size_t  CRawlog::size() const
{
	return m_seqOfActObs.size();
}

/*---------------------------------------------------------------
						getAsAction
  ---------------------------------------------------------------*/
CActionCollectionPtr  CRawlog::getAsAction( size_t index ) const
{
	MRPT_START

	if (index >=m_seqOfActObs.size())
		THROW_EXCEPTION("Index out of bounds")

	CSerializablePtr obj = m_seqOfActObs[index];

	if ( obj->GetRuntimeClass() == CLASS_ID(CActionCollection) )
			return CActionCollectionPtr( obj );
	else	THROW_EXCEPTION_CUSTOM_MSG1("Element at index %i is not a CActionCollection",(int)index);
	MRPT_END
}

/*---------------------------------------------------------------
						getAsObservation
  ---------------------------------------------------------------*/
CObservationPtr  CRawlog::getAsObservation( size_t index ) const
{
	MRPT_START

	if (index >=m_seqOfActObs.size())
		THROW_EXCEPTION("Index out of bounds")

	CSerializablePtr obj = m_seqOfActObs[index];

	if ( obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ) )
			return CObservationPtr( obj );
	else	THROW_EXCEPTION_CUSTOM_MSG1("Element at index %i is not a CObservation",(int)index);
	MRPT_END
}


/*---------------------------------------------------------------
						getAsGeneric
  ---------------------------------------------------------------*/
CSerializablePtr CRawlog::getAsGeneric( size_t index ) const
{
	MRPT_START
	if (index >=m_seqOfActObs.size())
		THROW_EXCEPTION("Index out of bounds")

	return m_seqOfActObs[index];
	MRPT_END
}

/*---------------------------------------------------------------
						getType
  ---------------------------------------------------------------*/
CRawlog::TEntryType CRawlog::getType( size_t index ) const
{
	MRPT_START
	if (index >=m_seqOfActObs.size())
		THROW_EXCEPTION("Index out of bounds")

	const CSerializablePtr &obj = m_seqOfActObs[index];

	if( obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ) )
		return etObservation;

	if( obj->GetRuntimeClass() == CLASS_ID(CActionCollection) )
		return etActionCollection;

	if( obj->GetRuntimeClass() == CLASS_ID(CSensoryFrame) )
		return etSensoryFrame;

	THROW_EXCEPTION("Object is not of any of the 3 allowed classes.");

	MRPT_END
}

/*---------------------------------------------------------------
						getAsObservations
  ---------------------------------------------------------------*/
CSensoryFramePtr  CRawlog::getAsObservations( size_t index ) const
{
	MRPT_START
	if (index >=m_seqOfActObs.size())
		THROW_EXCEPTION("Index out of bounds")

	CSerializablePtr obj = m_seqOfActObs[index];

	if ( obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CSensoryFrame) ))
			return CSensoryFramePtr( obj );
	else	THROW_EXCEPTION_CUSTOM_MSG1("Element at index %i is not a CSensoryFrame",(int)index);
	MRPT_END
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CRawlog::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		uint32_t	i,n;
		n = static_cast<uint32_t>( m_seqOfActObs.size() );
		out << n;
		for (i=0;i<n;i++)
			out << m_seqOfActObs[i];

		out << m_commentTexts;
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CRawlog::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t	i,n;

			clear();

			in >> n;
			m_seqOfActObs.resize(n);
			for (i=0;i<n;i++)
				m_seqOfActObs[i] = CSerializablePtr( in.ReadObject() );

			in >> m_commentTexts;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
						loadFromRawLogFile
  ---------------------------------------------------------------*/
bool  CRawlog::loadFromRawLogFile( const std::string &fileName )
{
	bool		keepReading = true;
	// Open for read.

	m_commentTexts.text.clear();

	CFileGZInputStream		fs(fileName);

	if (!fs.fileOpenCorrectly()) return false;

	// Clear first:
	clear();

	// OK: read objects:
	while (keepReading)
	{
		CSerializablePtr newObj;
		try
		{
			fs >> newObj;
            // Check type:
			if ( newObj->GetRuntimeClass() == CLASS_ID(CRawlog))
        	{
				// It is an entire object: Copy and finish:
				CRawlogPtr ao = CRawlogPtr( newObj );
				this->swap(*ao);
				return true;
            }
            else if ( newObj->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation)) )
            {
            	if (IS_CLASS(newObj,CObservationComment))
            	{
					CObservationCommentPtr o = CObservationCommentPtr(newObj);
					m_commentTexts = *o;
            	}
            	else
					m_seqOfActObs.push_back( newObj );
            }
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
            {
	        	m_seqOfActObs.push_back( newObj );
            }
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
            {
				m_seqOfActObs.push_back( newObj );
            }
			/** FOR BACKWARD COMPATIBILITY: CPose2D was used previously intead of an "ActionCollection" object
																				26-JAN-2006	*/
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CPose2D))
            {
				CPose2DPtr					poseChange = CPose2DPtr( newObj );
				CActionCollectionPtr	temp = CActionCollectionPtr( new CActionCollection() );
				CActionRobotMovement2D		action;
				CActionRobotMovement2D::TMotionModelOptions	options;
				action.computeFromOdometry( *poseChange, options);
				temp->insert( action );
				m_seqOfActObs.push_back( temp );
            }
			else
			{       // Unknown class:
				keepReading = false;
			}
		}
		catch (mrpt::utils::CExceptionEOF &)
		{	// EOF, just finish the loop
			keepReading = false;
		}
		catch (std::exception &e)
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

/*---------------------------------------------------------------
					remove
  ---------------------------------------------------------------*/
void  CRawlog::remove( size_t index )
{
	MRPT_START

	if (index >=m_seqOfActObs.size())
		THROW_EXCEPTION("Index out of bounds")

	m_seqOfActObs.erase( m_seqOfActObs.begin()+index );

	MRPT_END
}

/*---------------------------------------------------------------
					remove
  ---------------------------------------------------------------*/
void  CRawlog::remove( size_t first_index, size_t last_index )
{
	MRPT_START

	if (first_index >=m_seqOfActObs.size() || last_index>=m_seqOfActObs.size() )
		THROW_EXCEPTION("Index out of bounds")

	m_seqOfActObs.erase( m_seqOfActObs.begin()+first_index, m_seqOfActObs.begin()+last_index+1 );

	MRPT_END
}


/*---------------------------------------------------------------
						saveToRawLogFile
  ---------------------------------------------------------------*/
bool  CRawlog::saveToRawLogFile( const std::string &fileName ) const
{
	try
	{
		CFileGZOutputStream	f(fileName);

		if (!m_commentTexts.text.empty())
			f << m_commentTexts;

		for (size_t i=0;i<m_seqOfActObs.size();i++)
			f << *m_seqOfActObs[i];

		return true;
	}
	catch(...)
	{
		return false;
	}
}



/*---------------------------------------------------------------
					moveFrom
  ---------------------------------------------------------------*/
void CRawlog::moveFrom( CRawlog &obj)
{
	MRPT_START
	if (this == &obj) return;
	clear();
	m_commentTexts = obj.m_commentTexts;
	m_seqOfActObs = obj.m_seqOfActObs;
	obj.m_seqOfActObs.clear();
	obj.m_commentTexts.text.clear();
	MRPT_END
}

/*---------------------------------------------------------------
					swap
  ---------------------------------------------------------------*/
void CRawlog::swap( CRawlog &obj)
{
	if (this == &obj) return;
	m_seqOfActObs.swap(obj.m_seqOfActObs);
	std::swap(m_commentTexts, obj.m_commentTexts);
}

/*---------------------------------------------------------------
		getActionObservationPair
  ---------------------------------------------------------------*/
bool CRawlog::readActionObservationPair(
	CStream					&inStream,
	CActionCollectionPtr	&action,
	CSensoryFramePtr		&observations,
	size_t			& rawlogEntry )
{
	try
	{
		// Load pose change from the rawlog:
		action.clear_unique();
		while (!action)
		{
			CSerializablePtr obj;
			inStream >> obj;
			if (obj->GetRuntimeClass() == CLASS_ID( CActionCollection ) )
			{
				action = CActionCollectionPtr( obj );
			}
			else
			{
				obj.clear();
			}
			rawlogEntry++;
		};

		// Load sensory frame from the rawlog:
		observations.clear_unique();
		while (!observations)
		{
			CSerializablePtr obj;
			inStream >> obj;
			if (obj->GetRuntimeClass() == CLASS_ID( CSensoryFrame ) )
			{
				observations = CSensoryFramePtr(obj);
			}
			else
			{
				obj.clear();
			}
			rawlogEntry++;
		}
		return true;
	}
	catch ( CExceptionEOF &)
	{
		return false;
	}
	catch ( std::exception &e)
	{
		std::cerr << "[CRawlog::readActionObservationPair] Found exception:" << std::endl << e.what() << std::endl;
		return false;
	}
	catch(...)
	{
		std::cerr << "Untyped exception reading rawlog file!!" << std::endl;
		return false;
	}
}

/*---------------------------------------------------------------
		getActionObservationPairOrObservation
  ---------------------------------------------------------------*/
bool CRawlog::getActionObservationPairOrObservation(
	CStream					&inStream,
	CActionCollectionPtr	&action,
	CSensoryFramePtr		&observations,
	CObservationPtr			&observation,
	size_t			& rawlogEntry )
{
	try
	{
		// Load pose change from the rawlog:
		observations.clear_unique();
		observation.clear_unique();
		action.clear_unique();
		while (!action)
		{
			CSerializablePtr obj;
			inStream >> obj;
			if (IS_CLASS(obj,CActionCollection ) )
			{
				action = CActionCollectionPtr( obj );
			}
			else if (IS_DERIVED(obj,CObservation ) )
			{
				observation = CObservationPtr(obj);
				rawlogEntry++;
				return true;
			}
			else
				obj.clear();
			rawlogEntry++;
		};

		// Load sensory frame from the rawlog:
		observations.clear_unique();
		while (!observations)
		{
			CSerializablePtr obj;
			inStream >> obj;
			if (obj->GetRuntimeClass() == CLASS_ID( CSensoryFrame ) )
			{
				observations = CSensoryFramePtr(obj);
			}
			rawlogEntry++;
		}
		return true;
	}
	catch ( CExceptionEOF &)
	{
		return false;
	}
	catch ( std::exception &e)
	{
		std::cerr << "[CRawlog::readActionObservationPair] Found exception:" << std::endl << e.what() << std::endl;
		return false;
	}
	catch(...)
	{
		std::cerr << "Untyped exception reading rawlog file!!" << std::endl;
		return false;
	}
}



/*---------------------------------------------------------------
					findObservationsByClassInRange
  ---------------------------------------------------------------*/
void CRawlog::findObservationsByClassInRange(
	mrpt::system::TTimeStamp		time_start,
	mrpt::system::TTimeStamp		time_end,
	const mrpt::utils::TRuntimeClassId	*class_type,
	TListTimeAndObservations		&out_found,
	size_t							guess_start_position
	) const
{
	MRPT_START

	out_found.clear();

	if (m_seqOfActObs.empty()) return;

	// Find the first appearance of time_start:
	// ---------------------------------------------------
	TListObjects::const_iterator first = m_seqOfActObs.begin();
	const TListObjects::const_iterator last  = m_seqOfActObs.end();
	{
		// The following is based on lower_bound:
		size_t count, step;
		count = std::distance(first,last);
		while (count>0)
		{
			TListObjects::const_iterator it = first;
			step=count/2;
			std::advance(it,step);

			// The comparison function:
			TTimeStamp this_timestamp;
			if ( (*it)->GetRuntimeClass()->derivedFrom( CLASS_ID( CObservation ) ) )
			{
				CObservationPtr o = CObservationPtr (*it);
				this_timestamp = o->timestamp;
				ASSERT_(this_timestamp!=INVALID_TIMESTAMP);
			}
			else
				THROW_EXCEPTION("Element found which is not derived from CObservation");

			if (this_timestamp < time_start ) // *it < time_start
			{
				first=++it;
				count-=step+1;
			}
			else count=step;
		}
		// "first" is our guy
	}

	// Iterate until we get out of the time window:
	while (first!=last)
	{
		TTimeStamp this_timestamp;
		if ((*first)->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation)))
		{
			CObservationPtr o = CObservationPtr (*first);
			this_timestamp = o->timestamp;
			ASSERT_(this_timestamp!=INVALID_TIMESTAMP);

			if (this_timestamp<time_end)
			{
				if (o->GetRuntimeClass()->derivedFrom(class_type))
					out_found.insert( TTimeObservationPair(this_timestamp,o) );
			}
			else
			{
				break;	// end of time window!
			}
		}
		else
			THROW_EXCEPTION("Element found which is not derived from CObservation");

		first++;
	}

	MRPT_END
}


/*---------------------------------------------------------------
		getActionObservationPair
  ---------------------------------------------------------------*/
bool CRawlog::getActionObservationPair(
	CActionCollectionPtr  &action,
	CSensoryFramePtr      &observations,
	size_t	              &rawlogEntry ) const
{
	try
	{
		while (getType(rawlogEntry)!=CRawlog::etActionCollection)
		{
			rawlogEntry++;
		}
		action = getAsAction(rawlogEntry++);

		while (getType(rawlogEntry)!=CRawlog::etSensoryFrame)
		{
			rawlogEntry++;
		}
		observations = getAsObservations(rawlogEntry++);

		return true;
	}
	catch ( std::exception &)
	{
		return false;
	}
	catch(...)
	{
		std::cerr << "Untyped exception getting act-obs pair from rawlog!!" << std::endl;
		return false;
	}
}

/*---------------------------------------------------------------
		getCommentText
  ---------------------------------------------------------------*/
void CRawlog::getCommentText( std::string &t) const
{
	t = m_commentTexts.text;
}
std::string CRawlog::getCommentText() const
{
	return m_commentTexts.text;
}

/*---------------------------------------------------------------
		getCommentTextAsConfigFile
  ---------------------------------------------------------------*/
void CRawlog::getCommentTextAsConfigFile( mrpt::utils::CConfigFileMemory &memCfg ) const
{
	memCfg.setContent( m_commentTexts.text );
}

/*---------------------------------------------------------------
		setCommentText
  ---------------------------------------------------------------*/
void CRawlog::setCommentText( const std::string &t)
{
	m_commentTexts.text = t;
}

/*---------------------------------------------------------------
		detectImagesDirectory
  ---------------------------------------------------------------*/
std::string CRawlog::detectImagesDirectory(const std::string &str)
{
	const string rawlog_path = extractFileDirectory(str);
	string  temptative_img_path = rawlog_path + extractFileName(str) + string("_Images");
	if ( mrpt::system::fileExists(temptative_img_path))
		return temptative_img_path;
	else if ( mrpt::system::fileExists( temptative_img_path = (rawlog_path + extractFileName(str) + string("_images") )))
		return  temptative_img_path;
	else if ( mrpt::system::fileExists( temptative_img_path = (rawlog_path + extractFileName(str) + string("_IMAGES") )))
		return  temptative_img_path;
	else
		return  rawlog_path + "Images";
}
