/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

#include <mrpt/obs/CObservationRawDAQ.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;

#define WRITE_AIN_SAMPLES(_VECTOR_NAME, _FRMT, _TYPECAST) \
    if (!obs->_VECTOR_NAME.empty()) { \
        if (obs->AIN_interleaved) \
        { \
            for (i=0,tim=tim0;i<obs->_VECTOR_NAME.size()/obs->AIN_channel_count;i++,tim+=At) \
            { \
                ::fprintf(f_this,"%14.4f ", tim); \
                for (size_t j=0;j<obs->AIN_channel_count;j++) { \
                    ::fprintf(f_this,_FRMT " ", (_TYPECAST)obs->_VECTOR_NAME[i*obs->AIN_channel_count+j]); \
                    m_entriesSaved++; \
                } \
                ::fprintf(f_this, "\n"); \
            } \
        } \
        else \
        { \
            const size_t nSmpls = obs->_VECTOR_NAME.size()/obs->AIN_channel_count; \
            for (i=0,tim=tim0;i<nSmpls;i++,tim+=At) \
            { \
                ::fprintf(f_this,"%14.4f ", tim); \
                for (size_t j=0;j<obs->AIN_channel_count;j++) { \
                    ::fprintf(f_this,_FRMT " ", (_TYPECAST)obs->_VECTOR_NAME[i+ j*nSmpls]); \
                    m_entriesSaved++; \
                } \
                ::fprintf(f_this, "\n"); \
            } \
        } \
    }

#define WRITE_OTHER_SAMPLES(_VECTOR_NAME, _FRMT, _TYPECAST) \
    if (!obs->_VECTOR_NAME.empty()) { \
        for (i=0,tim=tim0;i<obs->_VECTOR_NAME.size();i++,tim+=At) { \
			::fprintf(f_this,"%14.4f ", tim); \
            ::fprintf(f_this,_FRMT " ", (_TYPECAST)obs->_VECTOR_NAME[i]); \
            m_entriesSaved++; \
        } \
        ::fprintf(f_this, "\n"); \
    }


// ======================================================================
//		op_export_rawdaq_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_rawdaq_txt)
{
	// A class to do this operation:
    class CRawlogProcessor_ExportRAWDAQ_TXT : public CRawlogProcessorOnEachObservation
	{
	protected:
		string	m_inFile;

		map<string, FILE*>	lstFiles;
		string				m_filPrefix;

	public:
        size_t				m_entriesSaved;


        CRawlogProcessor_ExportRAWDAQ_TXT(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose),
            m_entriesSaved(0)
		{
			getArgValue<string>(cmdline,"input",m_inFile);

			m_filPrefix =
				extractFileDirectory(m_inFile) +
				extractFileName(m_inFile);
		}

		// return false on any error.
		bool processOneObservation(CObservationPtr  &o)
		{
            if (!IS_CLASS(o, CObservationRawDAQ ) )
				return true;

            const CObservationRawDAQ* obs = CObservationRawDAQPtr(o).pointer();

			map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

			FILE *f_this;

			if ( it==lstFiles.end() )	// A new file for this sensorlabel??
			{
				const std::string fileName =
					m_filPrefix+
					string("_") +
					fileNameStripInvalidChars( obs->sensorLabel ) +
					string(".txt");

                VERBOSE_COUT << "Writing RAW DAQ TXT file: " << fileName << endl;

				f_this = lstFiles[ obs->sensorLabel ] = os::fopen( fileName.c_str(), "wt");
				if (!f_this)
					THROW_EXCEPTION_CUSTOM_MSG1("Cannot open output file for write: %s", fileName.c_str() );

				// The first line is a description of the columns:
				::fprintf(f_this,
					"%% "
					"%14s "				// Time
                    "%23s"
					"\n"
					,
					"Time",
                    "Measured values (meaning is channel-dependant)"
					);
			}
			else
				f_this = it->second;

            const double tim0 = mrpt::system::timestampTotime_t(obs->timestamp);
            const double At = obs->sample_rate!=0.0 ? 1.0/obs->sample_rate : 0.0;
            double tim;
            size_t i;

            // Analog IN channels:
            if (obs->AIN_channel_count>0)
			{
				// Save file:
                WRITE_AIN_SAMPLES(AIN_8bits,"%4u",unsigned int)
                WRITE_AIN_SAMPLES(AIN_16bits,"%6u",unsigned int)
                WRITE_AIN_SAMPLES(AIN_32bits,"%8u",unsigned int)
                WRITE_AIN_SAMPLES(AIN_float,"%9.5f",double)
                WRITE_AIN_SAMPLES(AIN_double,"%9.5f",double)
			}

            // Encoder channels:
            WRITE_OTHER_SAMPLES(CNTRIN_32bits,"%f",double)
            WRITE_OTHER_SAMPLES(CNTRIN_double,"%f",double)

			return true; // All ok
		}


		// Destructor: close files and generate summary files:
        ~CRawlogProcessor_ExportRAWDAQ_TXT()
		{
			for (map<string, FILE*>::const_iterator  it=lstFiles.begin();it!=lstFiles.end();++it)
			{
				os::fclose(it->second);
			}
			lstFiles.clear();
		} // end of destructor
	};

	// Process
	// ---------------------------------
    CRawlogProcessor_ExportRAWDAQ_TXT proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
    VERBOSE_COUT << "Number of records saved           : " << proc.m_entriesSaved << "\n";

}
