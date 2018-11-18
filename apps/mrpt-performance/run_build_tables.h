/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/datetime.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/serialization/CArchive.h>
#include <algorithm>

struct TPerfField
{
	string config_name;
	string file_path;
	vector<pair<string, double>> all_perf_data;
};

bool func_comp_entries(const TPerfField& a, const TPerfField& b)
{
	return a.config_name > b.config_name;
}

// ------------------------------------------------------
//                     run_build_tables
// ------------------------------------------------------
int run_build_tables()
{
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::system;
	using namespace mrpt::serialization;

	// Perf. results are in:
	//  PERF_DATA_DIR + mrpt::format("/perf-results-%i.%i.%i%s-%s-%ibit.dat"
	// Data is serializations of: vector<pair<string,double> >  all_perf_data;
	// // pair: description, time
	CDirectoryExplorer::TFileInfoList fils;
	CDirectoryExplorer::explore(PERF_DATA_DIR, FILE_ATTRIB_ARCHIVE, fils);
	CDirectoryExplorer::filterByExtension(fils, "dat");

	// make list of "perf-results-<SUFIX>" -> "whole path file"
	vector<TPerfField> lstConfigurations;

	for (auto& fil : fils)
	{
		if (!strStarts(fil.name, "perf-results")) continue;
		size_t p = fil.name.find(".dat");
		if (p == string::npos) continue;

		const string config_name = fil.name.substr(13, p - 13);

		TPerfField dat;
		dat.config_name = config_name;
		dat.file_path = fil.wholePath;

		CFileInputStream f(dat.file_path);
		auto arch = archiveFrom(f);
		arch >> dat.all_perf_data;
		lstConfigurations.push_back(dat);

		cout << " Read: " << setw(30) << config_name << " with "
			 << dat.all_perf_data.size() << " entries.\n";
	}

	std::sort(
		lstConfigurations.begin(), lstConfigurations.end(), func_comp_entries);

	ASSERT_(directoryExists(PERF_DATA_DIR + string("/perf-html/")));
	CFileOutputStream fo;
	// ====================================================
	//                  index.html
	// ====================================================
	const string out_fil_index = PERF_DATA_DIR + string(
													 "/perf-html/"
													 "index.html");
	cout << "Generating: " << out_fil_index << "...\n";
	fo.open(out_fil_index);
	fo.printf(
		"<html><head><title>mrpt-performance results - Index</title>\n"
		"<style type=\"text/css\"><!--\n"
		".my_rotated {\n"
		"     -moz-transform: rotate(-90deg);  /* FF3.5+ */\n"
		"       -o-transform: rotate(-90deg);  /* Opera 10.5 */\n"
		"  -webkit-transform: rotate(-90deg);  /* Saf3.1+, Chrome */\n"
		"             filter:  "
		"progid:DXImageTransform.Microsoft.BasicImage(rotation=-1."
		"57079632679490);  /* IE6,IE7 */\n"
		"         -ms-filter: "
		"\"progid:DXImageTransform.Microsoft.BasicImage(rotation=-1."
		"57079632679490)\"; /* IE8 */\n"
		"}\n"
		"--></style>\n"
		"</head>\n"
		"<body style=\"background-color:#E6E6FA\">\n\n");
	fo.printf("<h2>Performance reports</h2><hr>\n");

	fo.printf(
		"<br>\n"
		"This is a historic repository of performance tests executed against "
		"the MRPT libraries along time.\n"
		"Each entry represents a different MRPT version, different compilers "
		"or architectures."
		"Below follows the list of available performance tests (see also the "
		"<a href=\"#matrix\">comparison tables matrix</a>):<br>\n"
		"<ul>\n");

	for (const auto& P : lstConfigurations)
	{
		fo.printf(
			"<li>"
			"<a href=\"results_%s.html\">%s</a>"
			"</li>\n",
			P.config_name.c_str(), P.config_name.c_str());
	}

	fo.printf("</ul>\n");

	fo.printf("<a name=\"matrix\"> </a>\n");
	fo.printf("<h2>Comparison matrix</h2><hr>\n");

	// Build the matrix:
	// Only fills the lower triangle matrix entries.
	fo.printf("<div align=\"center\" ><table border=\"1\">\n");
	for (size_t i = 0; i < lstConfigurations.size(); i++)
	{
		const TPerfField& P1 = lstConfigurations[i];
		fo.printf(
			" <tr><td align=\"right\"><b>%s</b></td>", P1.config_name.c_str());

		for (size_t j = 0; j < lstConfigurations.size(); j++)
		{
			const TPerfField& P2 = lstConfigurations[j];
			//			if (j>i)
			//			{
			//				fo.printf("  <td border=\"0\"></td>\n");
			//				continue;
			//			}
			string txt;
			if (i == j)
			{
				txt = "&empty;";
			}
			else
			{
				txt = format(
					"<a href=\"comparison_%s_vs_%s.html\">&#9745;</a>",
					P1.config_name.c_str(), P2.config_name.c_str());
			}
			fo.printf("  <td align=\"center\">%s</td>\n", txt.c_str());
		}
		fo.printf(" </tr>\n");
	}
	// and the last row:
	fo.printf(" <tr>\n");
	fo.printf("  <td></td>\n");
	for (const auto& P1 : lstConfigurations)
	{
		// class=\"my_rotated\"
		fo.printf(
			"<td align=\"center\" width=\"20em\"><b><div >%s</div></b></td>",
			P1.config_name.c_str());
	}
	fo.printf(" </tr>\n");
	fo.printf("</table></div>\n");

	fo.printf(
		"<hr><small>Page generated automatically at %s with %s by "
		"'mrpt-performance'.</small>\n",
		mrpt::system::dateTimeLocalToString(now()).c_str(),
		MRPT_getVersion().c_str());
	fo.printf("</body></html>\n");
	fo.close();

	// ====================================================
	//        results_<cfg>.html
	// ====================================================
	for (const auto& P : lstConfigurations)
	{
		const string out_fil =
			PERF_DATA_DIR +
			format("/perf-html/results_%s.html", P.config_name.c_str());
		cout << "Generating: " << out_fil << "...\n";
		CFileOutputStream f(out_fil);

		f.printf(
			"<html><head><title>mrpt-performance results - Configuration: "
			"%s</title></head>\n"
			"<body style=\"background-color:#E6E6FA\">\n\n",
			P.config_name.c_str());

		f.printf(
			"<div align=\"center\"><h2>Results for: %s</h2></div>",
			P.config_name.c_str());
		f.printf("<hr> <br><br>\n");

		f.printf("<div align=\"center\"><table border=\"1\">\n");
		f.printf(
			"<tr> <td align=\"center\"><b>Test description</b></td> "
			"<td align=\"center\"><b>Execution time</b></td>"
			"<td align=\"center\"><b>Execution rate (Hz)</b></td> </tr>\n");

		for (const auto& j : P.all_perf_data)  // vector<pair<string,double> >
		{
			const double t = j.second;
			f.printf(
				"<tr> <td>%s</td> <td align=\"right\">%s</td> <td "
				"align=\"right\">%sHz</td>  </tr>\n",
				j.first.c_str(), mrpt::system::intervalFormat(t).c_str(),
				mrpt::system::unitsFormat(1.0 / t).c_str());
		}

		f.printf("</table></div>\n");
		f.printf("<p> &nbsp; </p>\n");

		f.printf(
			"<hr><small>Page generated automatically at %s with %s by "
			"'mrpt-performance'.</small>\n",
			mrpt::system::dateTimeLocalToString(now()).c_str(),
			MRPT_getVersion().c_str());
		f.printf("</body></html>\n");

	}  // for each perf

	// --------------------------------------
	//  COMPARISON TABLES
	// --------------------------------------
	for (size_t i = 0; i < lstConfigurations.size(); i++)
	{
		TPerfField P1 = lstConfigurations[i];
		for (size_t j = 0; j < lstConfigurations.size(); j++)
		{
			TPerfField P2 = lstConfigurations[j];
			if (j == i) continue;
			const string out_fil =
				PERF_DATA_DIR + format(
									"/perf-html/comparison_%s_vs_%s.html",
									P1.config_name.c_str(),
									P2.config_name.c_str());

			cout << "Generating: " << out_fil << "...\n";

			// Make sure P1 is the "more modern" version:
			//			if ( P1.config_name < P2.config_name )
			//				std::swap(P1,P2);

			// Convert P2 data into a std::map<> for search efficiency:
			map<string, double> P2_dat;
			for (auto& k : P2.all_perf_data) P2_dat[k.first] = k.second;

			CFileOutputStream f(out_fil);

			f.printf(
				"<html><head><title>mrpt-performance results - %s vs "
				"%s</title></head>\n"
				"<body style=\"background-color:#E6E6FA\">\n\n",
				P1.config_name.c_str(), P2.config_name.c_str());

			f.printf(
				"<div align=\"center\"><h2>Comparison: %s vs. %s</h2></div>",
				P1.config_name.c_str(), P2.config_name.c_str());
			f.printf("<hr> <br><br>\n");

			f.printf("<div align=\"center\"><table border=\"1\">\n");
			f.printf(
				"<tr> <td rowspan=\"2\" align=\"center\"><b>Test "
				"description</b></td> "
				"<td colspan=\"4\" align=\"center\"><b>%s / <div "
				"style=\"color:gray;\">%s</div></b></td>"
				"</tr>"
				"<tr>"
				"<td colspan=\"2\" align=\"center\"><b>Execution time</b></td>"
				"<td colspan=\"2\" align=\"center\"><b>Execution rate "
				"(Hz)</b></td> </tr>\n",
				P1.config_name.c_str(), P2.config_name.c_str());

			for (auto& k : P1.all_perf_data)  // vector<pair<string,double> >
			{
				const double t1 = k.second;
				const string test_name = k.first;

				const bool P2_has_this_one =
					P2_dat.find(test_name) != P2_dat.end();

				string str_secs, str_secs_res;
				string str_Hz, str_Hz_res;

				if (!P2_has_this_one)
				{
					str_secs = format(
						"%s / <div style=\"color:gray;\">X</div>",
						mrpt::system::intervalFormat(t1).c_str());
					str_Hz = format(
						"%sHz / <div style=\"color:gray;\">X</div>",
						mrpt::system::unitsFormat(1.0 / t1).c_str());
				}
				else
				{
					const double t2 = P2_dat[test_name];
					str_secs = format(
						"%s / <div style=\"color:gray;\">%s</div>",
						mrpt::system::intervalFormat(t1).c_str(),
						mrpt::system::intervalFormat(t2).c_str());

					const double At = t1 - t2;
					str_secs_res = format(
						"<div style=\"color:%s;\">%s %.02f%%</div>",
						At > 0 ? "red" : "blue", At > 0 ? "&Delta;" : "&nabla;",
						100 * At / t2);

					str_Hz = format(
						"%sHz / <div style=\"color:gray;\">%sHz</div>",
						mrpt::system::unitsFormat(1.0 / t1).c_str(),
						mrpt::system::unitsFormat(1.0 / t2).c_str());

					const double AHz = 1.0 / t1 - 1.0 / t2;
					str_Hz_res = format(
						"<div style=\"color:%s;\">%s %.02f%%</div>",
						AHz > 0 ? "blue" : "red",
						AHz > 0 ? "&Delta;" : "&nabla;",
						100 * AHz / (1.0 / t2));
				}

				f.printf(
					"<tr> <td>%s</td>"
					"<td align=\"right\">%s</td>"
					"<td align=\"center\">%s</td>"
					"<td align=\"right\">%s</td>"
					"<td align=\"center\">%s</td>"
					"</tr>\n",
					test_name.c_str(), str_secs.c_str(), str_secs_res.c_str(),
					str_Hz.c_str(), str_Hz_res.c_str());
			}

			f.printf("</table></div>\n");
			f.printf("<p> &nbsp; </p>\n");

			f.printf(
				"<hr><small>Page generated automatically at %s with %s by "
				"'mrpt-performance'.</small>\n",
				mrpt::system::dateTimeLocalToString(now()).c_str(),
				MRPT_getVersion().c_str());
			f.printf("</body></html>\n");
		}
	}

	return 0;
}
