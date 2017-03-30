#include "image_properties.h"
//ros libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
//c++ libraries
#include <iostream>
#include <vector>
#include <utility>
#include <math.h>

using namespace cv;
using namespace std;


namespace enc = sensor_msgs::image_encodings;
string WINDOW = "Occupancy-Gird";

nav_msgs::OccupancyGrid Final_Grid;

ros::Subscriber sub_Lanedata_left;
ros::Subscriber sub_Lanedata_right;

ros::Publisher pub_Lanedata;
//ros::Publisher map_pub,blown_map_pub;

std::vector<std::vector<Point> > Lane_points_left;
std::vector<std::vector<Point> > Lane_points_right;

Mat src_left, src_right,dst,labels;
Mat roi, final_grid;
int iterations_dilate = 1;
int ker_size_dilate = 2;
int area = 100;
double compactness;
Mat kernel_dilate;
std::vector<std::vector<Point> > contours;
std::vector<Vec4i> hierarchy;
std::vector<Point2f> Lanepoints;
void leftimage(const sensor_msgs::ImageConstPtr& original_image)
{
	Mat temp = Mat::zeros(Size(image_height, image_width), CV_8UC1);
	src_left = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::MONO8);
	}
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
        return;
    }

	cv::Mat src = cv_ptr->image;

	Point2f p[4] = {Point(0, 0), Point(639,0), Point(639, 439), Point(0, 439)};
	Point2f q[4] = {Point(129, 382), Point(248, 290), Point(260, 378), Point(225, 407)};

	Mat transform (3, 3, CV_32FC1);
	transform = getPerspectiveTransform(p, q);

	warpPerspective(src, roi, transform, Size(occ_grid_width, occ_grid_height));

	Lane_points_left.clear();

	Mat LeftROI;
	roi.copyTo(LeftROI);

	if (!final_grid.data)
	final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);

	final_grid = final_grid + LeftROI;

	Lane_points_left.clear();

	waitKey(1);

	// final_grid.release();

	src_left.release();
}


void rightimage(const sensor_msgs::ImageConstPtr& original_image)
{
	Mat temp = Mat::zeros(Size(image_height, image_width), CV_8UC1);
	src_right = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::MONO8);
	}
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
        return;
    }

	cv::Mat src = cv_ptr->image;

	Point2f p[4] = {Point(0, 0), Point(639,0), Point(639, 439), Point(0, 439)};
	Point2f q[4] = {Point(237, 275), Point(384, 383), Point(275, 410), Point(237, 379)};

	Mat transform (3, 3, CV_32FC1);
	transform = getPerspectiveTransform(p, q);

	warpPerspective(src, roi, transform, Size(occ_grid_width, occ_grid_height));

	Lane_points_right.clear();

	Mat RightROI;
	roi.copyTo(RightROI);

	if (!final_grid.data)
	final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);

	final_grid = final_grid + RightROI;

	Lane_points_right.clear();

	waitKey(1);

	// final_grid.release();

	src_right.release();
}

void endprocessing()
{
	iterations_dilate = getTrackbarPos("iterations_dilate", WINDOW);
	ker_size_dilate = getTrackbarPos("ker_size_dilate",WINDOW)*2 + 1;
	kernel_dilate = getStructuringElement(MORPH_ELLIPSE, Size (ker_size_dilate,ker_size_dilate), Point(-1,-1));
	dilate(final_grid, final_grid, kernel_dilate,Point(-1,-1), iterations_dilate);
	findContours(final_grid, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	area = getTrackbarPos("area", WINDOW);
	dst = Mat::zeros(final_grid.rows, final_grid.cols, CV_8UC1);
	for (int i = 0; i < contours.size(); ++i)
    {
        if (contourArea(contours[i]) > area)
        {
        	drawContours(dst, contours, i, Scalar(255),-1);
        }
        // for (int j=0; j < contours[i].size();j++)
        // {
        // 	Lanepoints.push_back(cv::Point2f( (float)contours[i][j].x, (float)contours[i][j].y ));
        // }
    }
 //    if (Lanepoints.size() > 0)
 //   	{
 //   		compactness = kmeans(Lanepoints, 2, labels, TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 2, KMEANS_RANDOM_CENTERS);
 //   		cout<<compactness<<endl;
	// }

	//Lanepoints.clear();
	Final_Grid.info.map_load_time = ros::Time::now();
    Final_Grid.header.frame_id = "lane";
    Final_Grid.info.resolution = (float)map_width/(100*(float)occ_grid_widthr);
    Final_Grid.info.width = 200;
    Final_Grid.info.height = 400;

    Final_Grid.info.origin.position.x = 0;
    Final_Grid.info.origin.position.y = 0;
    Final_Grid.info.origin.position.z = 0;

    Final_Grid.info.origin.orientation.x = 0;
    Final_Grid.info.origin.orientation.y = 0;
    Final_Grid.info.origin.orientation.z = 0;
    Final_Grid.info.origin.orientation.w = 1;

    for (int i = 0; i < dst.rows; ++i)
    {
        for (int j = 0; j < dst.cols; ++j)
        {
            if ( dst.at<uchar>(i,j) > 0)
            {
                //cout<<"r";
                Final_Grid.data.push_back(2);

            }
            else
                Final_Grid.data.push_back(dst.at<uchar>(i,j));
        }
    }
    pub_Lanedata.publish(Final_Grid);
    Final_Grid.data.clear();

}

int main(int argc, char **argv)
{
	double begin,end;
	
	ros::init(argc, argv, "Lane_Occupancy_Grid");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);

	image_transport::Publisher pub;
	
    
	image_transport::Subscriber sub_left = it.subscribe("/camera/image_raw1", 1, leftimage);
	image_transport::Subscriber sub_right = it.subscribe("/camera/image_raw3", 1, rightimage);

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	createTrackbar("iterations_dilate", WINDOW, &iterations_dilate, 10);
	createTrackbar("ker_size_dilate", WINDOW, &ker_size_dilate, 10);
	createTrackbar("area", WINDOW, &area, 200);
	//pub = it.advertise("/Lane_Occupancy_Grid", 1);
	pub_Lanedata = nh.advertise<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid", 1);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{	

		//begin = ros::Time::now().toSec();
		final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
		ros::spinOnce();
		final_grid = final_grid(Rect(150,0,200,400));
		//cvtColor(final_grid,final_grid,CV_GRAY2BGR);
		endprocessing();
		//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
		//pub.publish(msg);
		
		imshow(WINDOW, dst);
		//Final_Grid.data.clear();
		final_grid.release();
		loop_rate.sleep();
		//end = ros::Time::now().toSec();
		//cout<< end - begin <<endl;
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");
}