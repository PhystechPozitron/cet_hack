#ifndef LEDS_POSE_H_
#define LEDS_POSE_H_
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
using std::vector;
using std::string;

// parameters of the thresholding
const int H_min = 0;
const int H_max = 179;
const int S_min = 0;
const int S_max = 30;
const int V_min = 220;
const int V_max = 255;
const int dilate_size = 3;

// parameters of frame (in pixels, except smth_rate)
const int min_frame_perimeter = 40;
const int max_frame_perimeter = 800;
const double approx_accuracy_rate = 0.025;
const double min_side_rate = 0.08;

// parameters of leds (in pixels, except min(max)_led_circularity)
const int min_led_radius = 2;
const int max_led_radius = 20;
const double min_led_circularity = 0.1;
const double max_led_circularity = 1;
const int min_distance_to_border = 4;

namespace ledsPose {

	static void _thresholdImage(cv::InputArray input_image, cv::OutputArray output_image) {

		// threshold
		cv::Mat imageHSV;
		cv::cvtColor(input_image, imageHSV, cv::COLOR_BGR2HSV);
		cv::Mat thresholded_image;
		cv::inRange(imageHSV, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), thresholded_image);

		// erode and dilate
		cv::Mat ellipse;
		ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(min_led_radius, min_led_radius));
		cv::erode(thresholded_image, thresholded_image, ellipse);
		ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_size, dilate_size));
		cv::dilate(thresholded_image, thresholded_image, ellipse);

		thresholded_image.copyTo(output_image);
	}

	static bool _contoursComp(std::vector<cv::Point> a, std::vector<cv::Point> b) {
		float S_a = cv::contourArea(a);
		float S_b = cv::contourArea(b);
		return S_a < S_b;
	}

	static void _findAndSortContours(cv::InputArray thresholded_image, std::vector < std::vector <cv::Point> > &contours) {

		// find contours
		cv::Mat copied_image;
		thresholded_image.getMat().copyTo(copied_image);
		cv::findContours(copied_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		// sort contours by area
		std::sort(contours.begin(), contours.end(), _contoursComp);
	}

	static void _getContourCenter(std::vector <cv::Point> &contour, cv::Point2f &center) {
		auto ms = cv::moments(contour, false);
		center = cv::Point2f(ms.m10/ms.m00, ms.m01/ms.m00);
	}

	static double _length(cv::Point a, cv::Point b) {
		return sqrt( (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
	}

	static bool _checkFrameCandidate(std::vector <cv::Point> &frame_candidate) {

		// STEP 1: check the perimeter's value
		double perimeter = cv::arcLength(frame_candidate, true);
		if ((perimeter > max_frame_perimeter) || (perimeter < min_frame_perimeter )) return false;

		// STEP 2: approximate and check if the approximation is quadrangle
		std::vector <cv::Point> approx_curve;
		double approx_accuracy = approx_accuracy_rate * perimeter;
		cv::approxPolyDP(frame_candidate, approx_curve, approx_accuracy, true);
		if (approx_curve.size() != 4) return false;

		// STEP 3: check rate of size with minimal length to the perimeter of the quadrangle
		double min_side = max_frame_perimeter/4;
		for (unsigned int i = 0; i < 3; i++) {
			double current_side = _length(approx_curve[i], approx_curve[i + 1]);
			if (current_side <= min_side) min_side = current_side;
		}
		if (min_side/perimeter < min_side_rate) return false;

		// if all tests are passed
		return true;
	}

	static bool _isInside(std::vector <cv::Point> &contour_1, std::vector <cv::Point> &contour_2) {
		cv::Point2f center;
		_getContourCenter(contour_1, center);
		if (cv::pointPolygonTest(contour_2, center, false) != 1) return false;
		return true;
		}

	static bool _checkLedsCandidates(std::vector <std::vector <cv::Point>> &leds_candidates, std::vector <cv::Point> &frame_candidate) {

		double frame_area = cv::contourArea(frame_candidate);
		double led_area, led_perimeter, rate, circularity;
		for (unsigned int j = 0; j < 4; j++) {
			led_area = cv::contourArea(leds_candidates[j]);
			rate = led_area/frame_area;
			if ( (rate > 0.25) || (led_area > 3.14*max_led_radius*max_led_radius) || (led_area < 3.14*min_led_radius*min_led_radius) ) return false;
			led_perimeter = cv::arcLength(leds_candidates[j], true);
			circularity = 4*3.14*led_area/(led_perimeter*led_perimeter);
			if ((circularity < min_led_circularity)||(circularity > max_led_circularity)) return false;
		}
		return true;
	}

	static int _getNumber(double dx, double dy) {
		int i;
		if ((dx < 0) && (dy < 0)) i = 0;
		if ((dx >= 0) && (dy < 0)) i = 1;
		if ((dx >= 0) && (dy >= 0)) i = 2;
		if ((dx < 0) && (dy >= 0)) i = 3;
		return i;
	}

	static bool _checkAndNumerateCorners(cv::InputArray image, std::vector <cv::Point2f> &corners_in, std::vector <cv::Point2f> &corners_out){

		// STEP 1: numerate corners
		double cx = 0;
		double cy = 0;
		int num;
		auto numerated_corners = std::vector<cv::Point2f>(4, cv::Point2f(-1, -1));
		for (unsigned int i = 0; i < 4; i++) {
			cx += corners_in[i].x;
			cy += corners_in[i].y;
		}
		cx /= 4;
		cy /= 4;
		for (unsigned int i = 0; i < 4; i++) {
			num = _getNumber(corners_in[i].x - cx, corners_in[i].y - cy);
			numerated_corners[num] = corners_in[i];
		}

		// STEP 2: check if numeration was successful
		for (unsigned int j = 0; j < 4; j++) {
			if (numerated_corners[j].x == -1) return false;
		}

		// STEP 3: check convexity
		if (!cv::isContourConvex(numerated_corners)) return false;

		// STEP 4: check if not too close to the border of the image
		double perimeter = cv::arcLength(numerated_corners, true);
		int max_x = image.getMat().cols;
		int max_y = image.getMat().rows;
		if ((cx - perimeter/8 < min_distance_to_border) || (cx + perimeter/8 > max_x - min_distance_to_border)) return false;
		if ((cy - perimeter/8 < min_distance_to_border) || (cy + perimeter/8 > max_y - min_distance_to_border)) return false;

		// if all tests are passed
		corners_out = numerated_corners;
		return true;
	}

	static void _findCorners(cv::InputArray image, std::vector < std::vector <cv::Point>> &contours, std::vector <cv::Point2f> &corners) {
		std::vector <std::vector <cv::Point>> leds_candidates;
		std::vector <cv::Point2f> corners_candidates;
		cv::Point2f center;

		for (unsigned int i = 4; i < contours.size(); i++) {
			leds_candidates.clear();
			corners_candidates.clear();
			corners.clear();

			// check if contour is a frame
			if (!_checkFrameCandidate(contours[i])) continue;

			// find all contours inside current contour
			for (unsigned int j = 0; j < i; j++) {
				if (_isInside(contours[j], contours[i])) {
					leds_candidates.push_back(contours[j]);
				}
			}

			// check if there are four of them and everyone is led
			if ( (leds_candidates.size() != 4) || (!_checkLedsCandidates(leds_candidates, contours[i])) ) continue;
			for (unsigned int j = 0; j < 4; j++) {
				_getContourCenter(leds_candidates[j], center);
				corners_candidates.push_back(center);
			}

			// finally check corners_candidate and numerate corners
			if (!_checkAndNumerateCorners(image, corners_candidates, corners)) continue;
			break;
		}
	}

	static void _debug(cv::InputArray input_image, cv::InputArray thresholded_image, std::vector <std::vector <cv::Point>> contours, std::vector <cv::Point2f> corners) {
		for (unsigned int i = 0; i < contours.size(); i++) {
			cv::Scalar color;
			if (_checkFrameCandidate(contours[i])) {
				color = cv::Scalar(0, 0, 255);
			} else {
				color = cv::Scalar(255, 0, 0);
			}
			//cv::drawContours(input_image.getMat(), contours, i, color, 2);
		};
		if (corners.size() > 0) {
			for (unsigned int i = 0; i < corners.size(); i++) {
				auto center = cv::Point(corners[i]);
				cv::putText(input_image.getMat(), std::to_string(i), center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
			}
		}
		cv::namedWindow( "Image", CV_WINDOW_AUTOSIZE);
		cv::imshow("Image", input_image);
		cv::namedWindow( "BinImage", CV_WINDOW_AUTOSIZE);
		cv::imshow("BinImage", thresholded_image);
	}

	void detectLedMarker(cv::InputArray input_image, std::vector <cv::Point2f> &corners) {
		CV_Assert(!input_image.empty());

		// STEP 1: threshold image
		cv::Mat thresholded_image;
		_thresholdImage(input_image.getMat(), thresholded_image);

		// STEP 2: find all contours on the thresholded image and sort them by area
		std::vector < std::vector <cv::Point> > contours;
		_findAndSortContours(thresholded_image, contours);

		// STEP 3: find corners (centers of 4 leds) on the image
		_findCorners(thresholded_image, contours, corners);

		// debug
		//_debug(input_image, thresholded_image, contours, corners);
	}
}

#endif
