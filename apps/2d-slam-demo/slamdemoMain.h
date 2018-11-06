/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef SLAMDEMOMAIN_H
#define SLAMDEMOMAIN_H

//(*Headers(slamdemoFrame)
#include <wx/grid.h>
#include <wx/toolbar.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/menu.h>
#include <mrpt/otherlibs/mathplot/mathplot.h>
#include <wx/panel.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/timer.h>
//*)

#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/gui/WxUtils.h>

class slamdemoApp;

class slamdemoFrame : public wxFrame
{
	friend class slamdemoApp;

   public:
	slamdemoFrame(wxWindow* parent, wxWindowID id = -1);
	~slamdemoFrame() override;

   private:
	//(*Handlers(slamdemoFrame)
	void OnQuit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);
	void OnbtnResetClicked(wxCommandEvent& event);
	void OnbtnOneStepClicked(wxCommandEvent& event);
	void OnbtnRunClicked(wxCommandEvent& event);
	void OnbtnStopClicked(wxCommandEvent& event);
	void OnbtnRunBatchClicked(wxCommandEvent& event);
	void OntimSimulTrigger(wxTimerEvent& event);
	void OnConfigClicked(wxCommandEvent& event);
	void OnMenuSaveFilterState(wxCommandEvent& event);
	void OnMenuProfilerViewStats(wxCommandEvent& event);
	void OnMenuProfilerReset(wxCommandEvent& event);
	void OnmnuSaveLastDASelected(wxCommandEvent& event);
	void OnmnuItemSaveRawlogSelected(wxCommandEvent& event);
	//*)

	//(*Identifiers(slamdemoFrame)
	static const long ID_STATICTEXT1;
	static const long ID_PANEL3;
	static const long ID_CUSTOM1;
	static const long ID_STATICTEXT2;
	static const long ID_PANEL4;
	static const long ID_CUSTOM2;
	static const long ID_STATICTEXT10;
	static const long ID_PANEL5;
	static const long ID_CUSTOM3;
	static const long ID_STATICTEXT3;
	static const long ID_PANEL6;
	static const long ID_CUSTOM4;
	static const long ID_STATICTEXT4;
	static const long ID_PANEL7;
	static const long ID_GRID1;
	static const long ID_STATICTEXT5;
	static const long ID_PANEL9;
	static const long ID_CUSTOM7;
	static const long ID_STATICTEXT6;
	static const long ID_PANEL10;
	static const long ID_CUSTOM8;
	static const long ID_STATICTEXT7;
	static const long ID_PANEL11;
	static const long ID_CUSTOM9;
	static const long ID_PANEL1;
	static const long ID_STATICTEXT9;
	static const long ID_PANEL8;
	static const long ID_CUSTOM5;
	static const long ID_STATICTEXT11;
	static const long ID_PANEL14;
	static const long ID_CUSTOM6;
	static const long ID_STATICTEXT12;
	static const long ID_PANEL15;
	static const long ID_CUSTOM11;
	static const long ID_STATICTEXT13;
	static const long ID_PANEL16;
	static const long ID_CUSTOM12;
	static const long ID_PANEL2;
	static const long ID_STATICTEXT8;
	static const long ID_PANEL13;
	static const long ID_CUSTOM10;
	static const long ID_STATICTEXT14;
	static const long ID_PANEL17;
	static const long ID_CUSTOM13;
	static const long ID_PANEL12;
	static const long ID_NOTEBOOK1;
	static const long ID_MENUITEM1;
	static const long ID_MENUITEM2;
	static const long ID_MENUITEM3;
	static const long ID_MENUITEM6;
	static const long ID_MENUITEM4;
	static const long ID_MENUITEM5;
	static const long idMenuQuit;
	static const long ID_MENUITEM8;
	static const long ID_MENUITEM11;
	static const long ID_MENUITEM_SAVE_RAWLOG;
	static const long ID_MENUITEM9;
	static const long ID_MENUITEM10;
	static const long ID_MENUITEM7;
	static const long idMenuAbout;
	static const long ID_STATUSBAR1;
	static const long ID_TOOLBARITEM1;
	static const long ID_TOOLBARITEM2;
	static const long ID_BTNRUN;
	static const long ID_BTNSTOP;
	static const long ID_TOOLBARITEM4;
	static const long ID_TOOLBARITEM3;
	static const long ID_TOOLBARITEM6;
	static const long ID_TOOLBARITEM7;
	static const long ID_TOOLBAR1;
	static const long ID_TIMER1;
	//*)

	//(*Declarations(slamdemoFrame)
	mpWindow* plotErrorY;
	wxMenuItem* mnuStop;
	mpWindow* plotDaFP;
	wxToolBarToolBase* ToolBarItem5;
	wxPanel* Panel11;
	wxPanel* Panel1;
	wxPanel* Panel6;
	wxStaticText* lmIndCompat;
	mpWindow* plotErrorPhi;
	wxPanel* Panel7;
	wxStatusBar* StatusBar1;
	mpWindow* plotGT;
	wxStaticText* lbDaTN;
	mpWindow* plotDaTN;
	wxMenuItem* mnuItemSaveRawlog;
	wxToolBarToolBase* ToolBarItem6;
	mpWindow* plotIndivCompat;
	wxMenu* Menu3;
	mpWindow* plotDaTP;
	wxMenu* MenuItem4;
	wxPanel* Panel16;
	mpWindow* plotDaJCBB;
	wxPanel* Panel12;
	mpWindow* plotMap;
	wxPanel* Panel9;
	wxPanel* Panel8;
	wxStaticText* StaticText1;
	wxPanel* Panel10;
	wxToolBarToolBase* ToolBarItem7;
	wxPanel* Panel2;
	mpWindow* plotErrorX;
	wxToolBarToolBase* ToolBarItem2;
	wxStaticText* StaticText3;
	wxPanel* Panel4;
	wxStaticText* lbDatAssoc;
	wxMenuItem* MenuItem3;
	mpWindow* plotStatTime;
	wxPanel* Panel5;
	wxStaticText* lbMap;
	mpWindow* plotObs;
	wxToolBar* ToolBar1;
	wxPanel* Panel3;
	wxStaticText* lbGT;
	wxStaticText* StaticText7;
	wxPanel* Panel15;
	wxToolBarToolBase* ToolBarItem4;
	wxMenuItem* MenuItem5;
	wxPanel* Panel13;
	wxPanel* Panel14;
	wxStaticText* lbObs;
	wxStaticText* StaticText4;
	wxToolBarToolBase* ToolBarItem1;
	wxMenuItem* mnuRun;
	wxTimer timSimul;
	wxStaticText* StaticText5;
	wxMenuItem* mnuOneStep;
	wxStaticText* StaticText2;
	wxNotebook* Notebook1;
	wxToolBarToolBase* ToolBarItem3;
	wxMenuItem* MenuItem6;
	wxStaticText* lbDaTP;
	wxStaticText* StaticText6;
	wxGrid* gridDA;
	mpWindow* plotDaFN;
	wxMenuItem* mnuSaveLastDA;
	wxToolBarToolBase* ToolBarItem8;
	wxMenuItem* mnuRunBatch;
	wxPanel* Panel17;
	wxMenuItem* MenuItem8;
	//*)

	DECLARE_EVENT_TABLE()

	// This is needed in all classes having Eigen::Matrix'es or any other class
	// containing them:
	MRPT_MAKE_ALIGNED_OPERATOR_NEW

	// Layers in GT plot -------------
	mpFXYVector* m_lyGTMap;
	mpPolygon* m_lyGTRobot;
	mpPolygon* m_lyGTvisibleRange;

	// Layers in Map plot -------------
	mpPolygon* m_lyMapRobot;
	std::vector<mpLayer*> m_lyMapEllipses;

	// Layers in observation plot -------------
	mpPolygon* m_lyObsRobot;
	mpPolygon* m_lyObsvisibleRange;
	std::vector<mpLayer*> m_lyObsLMs;

	// Layers in IC plot -------------
	mpPolygon* m_lyICvisibleRange;
	std::vector<mpLayer*> m_lyIC_LMs;

	// Layers in errorX plot ----------
	mpFXYVector* m_lyERRX_err;
	mpFXYVector* m_lyERRX_boundUp;
	mpFXYVector* m_lyERRX_boundDown;

	// Layers in errorY plot ----------
	mpFXYVector* m_lyERRY_err;
	mpFXYVector* m_lyERRY_boundUp;
	mpFXYVector* m_lyERRY_boundDown;

	// Layers in errorPhi plot ----------
	mpFXYVector* m_lyERRPHI_err;
	mpFXYVector* m_lyERRPHI_boundUp;
	mpFXYVector* m_lyERRPHI_boundDown;

	// Layers in stats Time plot ----------
	mpFXYVector* m_lyStatTimes;

	// Layers in DA stats plot ----------
	mpFXYVector* m_lyDaFP;
	mpFXYVector* m_lyDaFN;
	mpFXYVector* m_lyDaTP;
	mpFXYVector* m_lyDaTN;

	mpFXYVector* m_lyDaJCBB;

	/** The main object which handles all 2D-EKF SLAM */
	mrpt::slam::CRangeBearingKFSLAM2D m_SLAM;

	/** The ground truth map, used to simulate observations */
	mrpt::maps::CLandmarksMap m_GT_map;

	/** The ground truth robot pose, used to simulate observations */
	mrpt::poses::CPose2D m_GT_pose;

	mrpt::obs::CObservationBearingRange m_lastObservation;
	/** Ground truth of the indices in the landmark map of the sensed landmarks.
	 */
	std::vector<size_t> m_lastObservation_GT_indices;

	/** Reconstructed map estimated_map_idx -> real_map_idx for the landmarks.
		 Used to evaluate the performance of data-association (D.A.)
	  */
	std::map<size_t, size_t> m_estimatedIDX2realIDX;
	/** At least inserted in the map once (or more if due to errors it's more
	 * than once). */
	std::set<size_t> m_realIDX_already_mapped;

	/** The output rawlog file to save simulated sensor obs (if enabled) */
	mrpt::io::CFileGZOutputStream m_rawlog_out_file;

	/** Historic data */
	struct THistoric
	{
		THistoric() = default;

		mrpt::poses::CPose2D GT_robot_pose;
		mrpt::poses::CPosePDFGaussian estimate_robot_pose;
		double run_time{0};
		uint16_t da_true_pos{0}, da_true_neg{0}, da_false_pos{0},
			da_false_neg{0};
		size_t jcbb_iters;
	};

	/** A registry of all data for the simulation, used to compute errors, etc..
	 */
	std::vector<THistoric, Eigen::aligned_allocator<THistoric>> m_historicData;

	/** Reset the simulator and re-generate the ground truth map
	 *  map_type can be:
	 *		- "1": The default map
	 *		- a plain text file name: A Nx2 matrix with the coordinates of the
	 *landmarks
	 */
	void resetSimulator(const std::string& map_type);

	/** Executes 1 step of the simulator (does NOT update the graphs) */
	void executeOneStep();

	/** Update all the plots with the latest data
	 */
	void updateAllGraphs(bool alsoGTMap = false);

	struct TSimulationOptions : public mrpt::config::CLoadableOptions
	{
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& source,
			const std::string& section) const override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		/** -1: random, other, use as seed */
		int random_seed{-1};

		/** the parameter to resetSimulator */
		std::string map_generator{"1"};

		/** # of landmarks in the random map */
		uint32_t randomMap_nLMs{70};

		mrpt::poses::CPose2D sensorOnTheRobot{0, 0, 0};

		double sensor_max_range{5};
		double sensor_min_range{0.50};
		double sensor_fov{mrpt::DEG2RAD(140.)};
		/** If false (default), data associatin must be done */
		bool sensorDistingishesLandmarks{false};

		/** Used to simulate the robot path */
		double path_square_len{8};
		/** The length (in meters) of each robot forward step. */
		double robot_step_length{0.3};

		/** sigma of the odometry errors in X/Y */
		double odometry_noise_std_xy{0.02};
		/** sigma of the odometry errors in PHI */
		double odometry_noise_std_phi{mrpt::DEG2RAD(0.2)};

		double uncert_overestim_odom{1.2};
		double uncert_overestim_sensor{1.2};

		bool show_map_real_correspondences{false};

		/** Mean and std of spurious readings per "sensor observation". */
		double spurious_count_mean{0}, spurious_count_std{0};
	};

	/** Options used in the simulator */
	TSimulationOptions options;
};

#endif  // SLAMDEMOMAIN_H
