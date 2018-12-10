#include <ros/ros.h>
#include <stdio.h>
#include <leds_pose.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <iostream>

using std::string;
using std::vector;

//---- class description ----//

class rpsPoseClass {

private:
	cv::Ptr<cv::aruco::Dictionary> dict;
    cv::Ptr<cv::aruco::DetectorParameters> param;
	cv::Ptr<cv::aruco::Board> board;
	cv::Vec3d pose, orient;
	int markers_detected;
	int fail_cnt, max_fail_cnt;
	int use_led_marker;
	int number_x, number_y, first_marker;
	float side, dist_x, dist_y;

	image_transport::CameraSubscriber img_sub;
	image_transport::Publisher img_pub;
	ros::Publisher board_pub;
	ros::Publisher pose_pub;

	void createBoard();
	void publishPoseMsg(int); 
	void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr&, cv::Mat&, cv::Mat&);
	void detect(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);

public:
	rpsPoseClass(int argc, char **argv) {
		ros::init(argc, argv, "rps_pose");
		ros::NodeHandle nh;
		ROS_INFO("initializing rps_pose...");

		nh.param("rps_pose/use_led_marker", use_led_marker, 0);
		nh.param("rps_pose/number_x", number_x, 1);
		nh.param("rps_pose/number_y", number_y, 1);
		nh.param("rps_pose/first_marker", first_marker, 0);
		nh.param<float>("rps_pose/side", side, 0.08);
		nh.param<float>("rps_pose/dist_x", dist_x, 0);
		nh.param<float>("rps_pose/dist_y", dist_y, 0);
		if (!use_led_marker) { 
			dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
			param = cv::aruco::DetectorParameters::create();
			board = cv::makePtr<cv::aruco::Board>();
			board_pub = nh.advertise<sensor_msgs::Image>("/RPS/board_image", 1, true);
			createBoard();
		}

		fail_cnt = -1;
		nh.param("rps_pose/max_fail_cnt", max_fail_cnt, 5);

		image_transport::ImageTransport it(nh); 
		img_sub = it.subscribeCamera("/main_camera/throttled", 1, &rpsPoseClass::detect, this);
		
		img_pub = it.advertise("/RPS/debug", 1);
		pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/RPS/pose", 1);
		ROS_INFO("rps_pose inited");
	}
};

//---- methods ----//

void rpsPoseClass::createBoard() {
	float max_y = side + (number_y - 1)*(side + dist_y);
	std::vector<int> markers_ids;
	markers_ids.resize(number_x*number_y);
	for (int i = 0; i < number_x*number_y; i++) {
		markers_ids.at(i) =  first_marker + i;
	}
	board->ids = markers_ids;

	for (int i = 0; i < number_y; i++ ) {
		for (int j = 0; j < number_x; j++) {
			std::vector<cv::Point3f> corners;
			corners.resize(4);
			corners[0] = cv::Point3f(j*(side + dist_x), max_y - i*(side + dist_y), 0);
			corners[1] = cv::Point3f(j*(side + dist_x) + side, max_y - i*(side + dist_y), 0);
			corners[2] = cv::Point3f(j*(side + dist_x) + side, max_y - i*(side + dist_y) - side, 0);
			corners[3] = cv::Point3f(j*(side + dist_x), max_y - i*(side + dist_y) - side, 0);
			board->objPoints.push_back(corners);
		} 
	}
	board->dictionary = dict;

	// debug
	cv::Mat board_image;
	cv_bridge::CvImage board_msg;
	cv::aruco::drawPlanarBoard(board, cv::Size(2000, 2000), board_image, 50, 1);
	cv::cvtColor(board_image, board_image, CV_GRAY2BGR);

	board_msg.image = board_image;
	board_msg.encoding = sensor_msgs::image_encodings::BGR8;
	board_pub.publish(board_msg.toImageMsg());

}

void rpsPoseClass::detect(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cinfo) {
	
	// detect
	cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::Mat camera_matrix(3, 3, CV_64F);
	cv::Mat dist_coeffs(8, 1, CV_64F);
	cv_bridge::CvImage img_msg;
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;
	cv::Vec3d rvec, tvec;

	parseCameraInfo(cinfo, camera_matrix, dist_coeffs);
	if (!use_led_marker) {

		cv::aruco::detectMarkers(image, dict, corners, ids, param);

		if (ids.size() > 0) {
			markers_detected = cv::aruco::estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec);
		} 
		else {
			markers_detected = 0;
		}

	} else {
		std::vector <cv::Point2f> led_corners;
		std::vector <cv::Vec3d> rvecs, tvecs;
		ledsPose::detectLedMarker(image, led_corners);

		if (led_corners.size() > 0) {
			corners.push_back(led_corners);
			ids.push_back(-1);
			cv::aruco::estimatePoseSingleMarkers(corners, side, camera_matrix, dist_coeffs, rvecs, tvecs);
			if (rvecs.size() > 0) {
				rvec = rvecs[0];
				tvec = tvecs[0];
				markers_detected = 1;
			} else {
				markers_detected = 0;
			}
		} else {
			markers_detected = 0;
		}
	}

	// decide if publish or not
	int valid;
	if (markers_detected > 0) {
		fail_cnt = 0;
		pose = -tvec;
		orient = rvec;
		valid = 1;
	} else {
		if (fail_cnt != -1) {
			valid = 1;
			fail_cnt++;
			if (fail_cnt > max_fail_cnt) {
				fail_cnt = -1;
				valid = 0;
			}
		} else {
			valid = 0;
		}
	}
	publishPoseMsg(valid);

	// debug
	if (img_pub.getNumSubscribers() > 0) {
		cv::aruco::drawDetectedMarkers(image, corners, ids);	
		if (markers_detected > 0) {
			cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.3);
		}
		img_msg.header.frame_id = msg->header.frame_id;
		img_msg.header.stamp = ros::Time::now();
		img_msg.encoding = sensor_msgs::image_encodings::BGR8;
		img_msg.image = image;
		img_pub.publish(img_msg.toImageMsg());
	}
}

void rpsPoseClass::parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo, cv::Mat &camera_matrix, cv::Mat &dist_coeffs) {
    for (int i = 0; i < 3; i++ ) {
        for (int j = 0; j < 3; j++ ) {
            camera_matrix.at<double>(i, j) = cinfo->K[3 * i + j];
        }
    }
    for (int k = 0; k < cinfo->D.size(); k++ ) {
        dist_coeffs.at<double>(k) = cinfo->D[k];
    }
}

//void rpsPoseClass::calculateAndPublish() {
//}

void rpsPoseClass::publishPoseMsg(int valid) {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.stamp = ros::Time::now();
	if (valid) {
		pose_msg.header.frame_id = "main_camera";
		pose_msg.pose.position.x = pose[0];
		pose_msg.pose.position.y = pose[1];
		pose_msg.pose.position.z = pose[2];
		pose_msg.pose.orientation.x = orient[0];
		pose_msg.pose.orientation.y = orient[1];
		pose_msg.pose.orientation.z = orient[2];

	} else {
		pose_msg.header.frame_id = "";
	}

	pose_pub.publish(pose_msg);
}

//---- main ----//
int main(int argc, char **argv) {
	rpsPoseClass rps_pose(argc, argv);
	ros::spin();
	return 0;
}