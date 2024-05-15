/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include <mrpt/system/string_utils.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

// ======================================================================
//		op_remap_timestamps
// ======================================================================
DECLARE_OP_FUNCTION(op_remap_timestamps)
{
  // A class to do this operation:
  class CRawlogProcessor_RemapTimestamps : public CRawlogProcessorOnEachObservation
  {
   protected:
    TOutputRawlogCreator outrawlog;
    const double m_a, m_b;
    const std::set<std::string> m_labels;  //!< empty: all sensors
    size_t m_changes = 0;

   public:
    CRawlogProcessor_RemapTimestamps(
        CFileGZInputStream& in_rawlog,
        TCLAP::CmdLine& cmdline,
        bool Verbose,
        double a,
        double b,
        const std::set<std::string>& labels) :
        CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose),
        m_a(a),
        m_b(b),
        m_labels(labels)
    {
      VERBOSE_COUT << mrpt::format("Applying timestamps remap a*t+b with: a=%f b=%f", m_a, m_b)
                   << std::endl;

      std::string sLog = "Applying to sensor labels: ";
      if (m_labels.empty())
      {
        sLog += " (all)\n";
      }
      else
      {
        for (const auto& l : m_labels)
        {
          sLog += "'";
          sLog += l;
          sLog += "', ";
        }
        sLog += "\n";
      }
      VERBOSE_COUT << sLog;
    }

    ~CRawlogProcessor_RemapTimestamps()
    {
      VERBOSE_COUT << "Changed objects: " << m_changes << "\n";
    }

    bool checkSensorLabel(const CObservation::Ptr& obs)
    {
      if (m_labels.empty()) return true;
      return m_labels.count(obs->sensorLabel) != 0;
    }

    bool processOneObservation(CObservation::Ptr& obs) override
    {
      // does it apply?
      if (!checkSensorLabel(obs)) return true;

      // T_NEW = a * T_OLD + b
      const double t = mrpt::Clock::toDouble(obs->timestamp);
      const double t_new = m_a * t + m_b;
      obs->timestamp = mrpt::Clock::fromDouble(t_new);

      m_changes++;
      return true;
    }

    // This method can be reimplemented to save the modified object to
    // an output stream.
    void OnPostProcess(
        mrpt::obs::CActionCollection::Ptr& actions,
        mrpt::obs::CSensoryFrame::Ptr& SF,
        mrpt::obs::CObservation::Ptr& obs) override
    {
      ASSERT_((actions && SF) || obs);
      if (actions)
        (*outrawlog.out_rawlog) << actions << SF;
      else
        (*outrawlog.out_rawlog) << obs;
    }
  };

  string sAB_params;
  if (!getArgValue<string>(cmdline, "remap-timestamps", sAB_params) || sAB_params.empty())
    throw std::runtime_error(
        "remap-timestamps: This operation needs two arguments in the "
        "format 'a;b'.");

  vector<string> sAB_tokens;
  mrpt::system::tokenize(sAB_params, " ;", sAB_tokens);
  if (sAB_tokens.size() != 2)
    throw std::runtime_error(
        "remap-timestamps: This operation needs two arguments in the "
        "format 'a;b'.");

  const double a = atof(sAB_tokens[0].c_str());
  const double b = atof(sAB_tokens[1].c_str());

  string filter_labels;
  getArgValue<string>(cmdline, "select-label", filter_labels);

  std::vector<std::string> lbs;
  mrpt::system::tokenize(filter_labels, ",", lbs);

  std::set<std::string> applyLabels;
  for (const auto& l : lbs) applyLabels.insert(l);

  // Process
  // ---------------------------------
  CRawlogProcessor_RemapTimestamps proc(in_rawlog, cmdline, verbose, a, b, applyLabels);
  proc.doProcessRawlog();

  // Dump statistics:
  // ---------------------------------
  VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
}
