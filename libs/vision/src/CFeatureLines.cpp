/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/vision/CFeatureLines.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/vision/utils.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt::vision;
using namespace std;

#if MRPT_HAS_OPENCV

static void extractLines_CannyHough(
	const cv::Mat& canny_image, const std::vector<cv::Vec2f> lines,
	std::vector<cv::Vec4i>& segments, size_t threshold)
{
	// Some variables to change the coordinate system from polar to cartesian
	double rho, theta;
	double cosTheta, sinTheta;
	double m, c, cMax;

	// Variables to define the two extreme points on the line, clipped on the
	// window The constraint is x <= xF
	int x, y, xF, yF;

	segments.clear();

	// For each line
	for (size_t n(0); n < lines.size(); ++n)
	{
		// OpenCV implements the Hough accumulator with theta from 0 to PI, thus
		// rho could be negative We want to always have rho >= 0
		if (lines[n][0] < 0)
		{
			rho = -lines[n][0];
			theta = lines[n][1] - CV_PI;
		}
		else
		{
			rho = lines[n][0];
			theta = lines[n][1];
		}

		if (rho == 0 && theta == 0)
		{  // That case appeared at least once, so a test is needed
			continue;
		}

		// Since the step for theta in cv::HoughLines should not be too small,
		// theta should exactly be 0 (and the line vertical) when this condition
		// is true
		if (fabs(theta) < 0.00001)
		{
			x = xF = static_cast<int>(rho + 0.5);
			y = 0;
			yF = canny_image.rows - 1;
		}
		else
		{
			// Get the (m, c) slope and y-intercept from (rho, theta), so that y
			// = mx + c <=> (x, y) is on the line
			cosTheta = cos(theta);
			sinTheta = sin(theta);
			m = -cosTheta / sinTheta;  // We are certain that sinTheta != 0
			c = rho * (sinTheta - m * cosTheta);

			// Get the two extreme points (x, y) and (xF, xF) for the line
			// inside the window, using (m, c)
			if (c >= 0)
			{
				if (c < canny_image.rows)
				{
					// (x, y) is at the left of the window
					x = 0;
					y = static_cast<int>(c);
				}
				else
				{
					// (x, y) is at the bottom of the window
					y = canny_image.rows - 1;
					x = static_cast<int>((y - c) / m);
				}
			}
			else
			{
				// (x, y) is at the top of the window
				x = static_cast<int>(-c / m);
				y = 0;
			}
			// Define the intersection with the right side of the window
			cMax = m * (canny_image.cols - 1) + c;
			if (cMax >= 0)
			{
				if (cMax < canny_image.rows)
				{
					// (xF, yF) is at the right of the window
					xF = canny_image.cols - 1;
					yF = static_cast<int>(cMax);
				}
				else
				{
					// (xF, yF) is at the bottom of the window
					yF = canny_image.rows - 1;
					xF = static_cast<int>((yF - c) / m);
				}
			}
			else
			{
				// (xF, yF) is at the top of the window
				xF = static_cast<int>(-c / m);
				yF = 0;
			}
		}

		// Going through the line using the Bresenham algorithm
		// dx1, dx2, dy1 and dy2 are increments that allow to be successful for
		// each of the 8 octants (possible directions while iterating)
		bool onSegment = false;
		int memory;
		int memoryX = 0, memoryY = 0;
		int xPrev = 0, yPrev = 0;
		size_t nbPixels = 0;

		int w = xF - x;
		int h = yF - y;
		int dx1, dy1, dx2, dy2 = 0;

		int longest, shortest;
		int numerator;

		if (w < 0)
		{
			longest = -w;
			dx1 = -1;
			dx2 = -1;
		}
		else
		{
			longest = w;
			dx1 = 1;
			dx2 = 1;
		}

		if (h < 0)
		{
			shortest = -h;
			dy1 = -1;
		}
		else
		{
			shortest = h;
			dy1 = 1;
		}

		// We want to know whether the direction is more horizontal or vertical,
		// and set the increments accordingly
		if (longest <= shortest)
		{
			memory = longest;
			longest = shortest;
			shortest = memory;
			dx2 = 0;
			if (h < 0)
			{
				dy2 = -1;
			}
			else
			{
				dy2 = 1;
			}
		}

		numerator = longest / 2;

		// cout << n << " numerator " << numerator << endl;
		for (int i(0); i <= longest; ++i)
		{
			// For each pixel, we don't want to use a classic "plot", but to
			// look into canny_image for a black or white pixel
			if (onSegment)
			{
				if (canny_image.at<char>(y, x) == 0 || i == longest)
				{
					// We are leaving a segment
					onSegment = false;
					if (nbPixels >= threshold)
					{
						segments.emplace_back(memoryX, memoryY, xPrev, yPrev);
						// cout << "new segment " << segments.back() << endl;
					}
				}
				else
				{
					// We are still on a segment
					++nbPixels;
				}
			}
			else if (canny_image.at<char>(y, x) != 0)
			{
				// We are entering a segment, and keep this first position in
				// (memoryX, memoryY)
				onSegment = true;
				nbPixels = 0;
				memoryX = x;
				memoryY = y;
			}

			// xPrev and yPrev are used when leaving a segment, to keep in
			// memory the last pixel on it
			xPrev = x;
			yPrev = y;

			// Next pixel using the condition of the Bresenham algorithm
			numerator += shortest;
			if (numerator >= longest)
			{
				numerator -= longest;
				x += dx1;
				y += dy1;
			}
			else
			{
				x += dx2;
				y += dy2;
			}
		}
	}
}
#endif

void CFeatureLines::extractLines(
	const mrpt::img::CImage& img_in,
	std::vector<std::array<int, 4>>& segments_out, size_t threshold,
	const bool display)
{
#if MRPT_HAS_OPENCV

	using namespace cv;
	using namespace std;
	const Mat image = cv::cvarrToMat(img_in.getAs<IplImage>());

	// Canny edge detector
	cv::Mat canny_img;
	int lowThreshold = 150;
	// int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;
	cv::Canny(
		image, canny_img, lowThreshold, lowThreshold * ratio,
		kernel_size);  // 250, 600 // CAUTION: Both thresholds depend on the
	// input image, they might be a bit hard to set because
	// they depend on the strength of the gradients
	//            cv::namedWindow("Canny detector", cv::WINDOW_AUTOSIZE);
	//            cv::imshow("Canny detector", canny_img);
	//            cv::waitKey(0);
	//            cv::destroyWindow("Canny detector");

	// Get lines through the Hough transform

	// CvMat matCannyStub, *matCannyImg = (CvMat*)canny_img.getAs<IplImage>();
	// CV_CALL(matCannyImg = cvGetMat(canny_img.getAs<IplImage>(),
	// &matCannyStub));

	std::vector<cv::Vec2f> lines;
	cv::HoughLines(
		canny_img, lines, 1, CV_PI / 180.0,
		threshold);  // CAUTION: The last parameter depends on the input image,
	// it's the smallest number of pixels to consider a line in
	// the accumulator
	//    double minLineLength=50, maxLineGap=5;
	//    cv::HoughLinesP(canny_img, lines, 1, CV_PI / 180.0, threshold,
	//    minLineLength, maxLineGap); // CAUTION: The last parameter depends on
	//    the input image, it's the smallest number of pixels to consider a line
	//    in the accumulator
	// std::cout << lines.size() << " lines detected" << std::endl;

	// Possible dilatation of the canny detector
	// Useful when the lines are thin and not perfectly straight
	cv::Mat filteredCanny;
	// Choose whether filtering the Canny or not, for thin and non-perfect edges
	/*filteredCanny = canny_img.clone();*/
	cv::dilate(canny_img, filteredCanny, cv::Mat());
	//    cv::namedWindow("Filtered Canny detector");
	//    cv::imshow("Filtered Canny detector", filteredCanny);
	//    cv::waitKey(0);
	//    cv::destroyWindow("Filtered Canny detector");

	// Extracting segments (pairs of points) from the filtered Canny detector
	// And using the line parameters from lines
	std::vector<cv::Vec4i> segments;
	extractLines_CannyHough(filteredCanny, lines, segments, threshold);

	// Force the segments to have a predefined order (e1y <= e2y)
	for (auto line = begin(segments); line != end(segments); ++line)
		if ((*line)[1] == (*line)[3])
		{
			if ((*line)[0] > (*line)[1])
				*line =
					cv::Vec4i((*line)[2], (*line)[3], (*line)[0], (*line)[1]);
		}
		else if ((*line)[1] < (*line)[3])
			*line = cv::Vec4i((*line)[2], (*line)[3], (*line)[0], (*line)[1]);

	// convert cv->STL:
	segments_out.resize(segments.size());
	for (unsigned i = 0; i < segments.size(); i++)
		for (int k = 0; i < 4; k++) segments_out[i][k] = segments[i][k];

	// Display 2D segments
	if (display)
	{
		cv::Mat image_lines;
		image.convertTo(image_lines, CV_8UC1, 1.0 / 2);
		for (auto line = begin(segments); line != end(segments); ++line)
		{
			// cout << "line: " << (*line)[0] << " " << (*line)[1] << " " <<
			// (*line)[2] << " " << (*line)[3] << " \n";
			// image.convertTo(image_lines, CV_8UC1, 1.0 / 2);
			cv::line(
				image_lines, cv::Point((*line)[0], (*line)[1]),
				cv::Point((*line)[2], (*line)[3]), cv::Scalar(255, 0, 255), 1);
			cv::circle(
				image_lines, cv::Point((*line)[0], (*line)[1]), 3,
				cv::Scalar(255, 0, 255), 3);
			cv::putText(
				image_lines, string(to_string(distance(begin(segments), line))),
				cv::Point(
					((*line)[0] + (*line)[2]) / 2,
					((*line)[1] + (*line)[3]) / 2),
				0, 1.2, cv::Scalar(200, 0, 0), 3);
			// cv::imshow("lines", image_lines); cv::moveWindow("lines",
			// 20,100+700);  cv::waitKey(0);
		}
		// cv::imwrite("/home/efernand/lines.png", image_lines);
		cv::imshow("lines", image_lines);
		cv::moveWindow("lines", 20, 100 + 700);
		cv::waitKey(0);
	}
#else
	THROW_EXCEPTION("Requires building MRPT with OpenCV support")
#endif
}
