#include <iostream>
#include <cstring>
#include <string>
#include <cerrno>
#include <map>
#include <fstream>

#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils.h>

#pragma once

typedef std::pair<std::string, std::ifstream*> file_in;
typedef std::pair<std::string, std::ofstream*> file_out;

class CParams: public mrpt::utils::CLoadableOptions {

  public:

    CParams();
    CParams(mrpt::utils::CConfigFile &config_file);
    ~CParams();
    virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,
                                    const std::string &section);
    virtual void dumpToConsole() const;

  private:
    bool pose_graph_only;

    file_in laser_file;

    std::string log_output_dir;
    float  log_frequency;

    bool debug;
    file_out debug_logfile;

    bool robot_pose_log;
    file_out robot_pose_logfile;


};

CParams::CParams()
{}

CParams::CParams(mrpt::utils::CConfigFile &config_file)
{

  std::cout << "In CParams::CParams() ctor" << std::endl;

  // read all the variables at once.
  loadFromConfigFile(config_file, "GRAPHSLAM_options");

}
CParams::~CParams()
{}
void CParams::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,
    const std::string &section)
{
  std::cout << "CParams::loadFromConfigFile" << std::endl;

  std::string tmp_str;
  std::ifstream *f_in = new std::ifstream;
  std::ofstream *f_out = new std::ofstream;

  //prepare fstreams to throw if failbit gets set
  std::ios_base::iostate exceptionMask = f_in->exceptions() | std::ios::failbit;
  f_in->exceptions(exceptionMask);
  std::cout << "kalimera" << std::endl;
  exceptionMask = f_out->exceptions() | std::ios::failbit;
  f_out->exceptions(exceptionMask);
  std::cout << "kalinuxta" << std::endl;

  pose_graph_only = source.read_bool(section, "pose_graph_only", /*default=*/ false);
  std::cout << "pose_graph_only" << pose_graph_only << std::endl;

  tmp_str = source.read_string(section, "laser_file", /*default=*/ "noVal");
  try {
    f_in->open(tmp_str.c_str(), std::ifstream::in);
    laser_file.first = tmp_str;
    laser_file.second = f_in;
  }
  catch (std::ios_base::failure& e) {
    std::cerr << "I/O Exception: " << strerror(errno);

  }
  catch (std::exception &e) {
    std::cerr << "Exception:" << e.what() << '\n';
  }

}


void CParams::dumpToConsole() const
{
  std::cout << "Open files: " << std::endl;

  std::cout << "laser_file: " << laser_file.first << std::endl;

}


