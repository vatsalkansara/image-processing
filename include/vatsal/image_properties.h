#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include <cmath>

#ifndef IMAGE_PROPERTIES_H
#define IMAGE_PROPERTIES_H
	
	const int image_width = 640;
	const int image_height = 480;

	const int occ_grid_width = 500;
	const int occ_grid_height = 500;

	const int occ_grid_widthr = 200;
	const int occ_grid_heightr = 400;	

	const int map_width = 1000;
	const int map_length = 2000;

	const int no_of_sections = 4;

	int Calib_Bottom_Left[no_of_sections] = {187, 171, 161, 131};
	int Calib_Bottom_Right[no_of_sections] = {184, 154, 149, 123};

	float Calib_Begin_Dist_y[no_of_sections] = {401.0, 227.0, 126.0, 80.0};
	
	float Calib_Dist_x[no_of_sections] = {47.0, 47.0, 47.0, 47.0};
	float Calib_Dist_y[no_of_sections] = {50.0, 53.0, 51.5, 45.5};

	float Calib_Pix_x[no_of_sections] = {(765.0 - 691.0), (801.0 - 679.0), (824.0 - 647.0), (853.0 - 621.0)};
	float Calib_Pix_y[no_of_sections] = {(167.0 - 146.0), (138.0 - 80.0), (152.0 - 35.0), (158.0 - 0.0)};

#endif
