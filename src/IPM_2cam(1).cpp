#include "image_properties.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;
string WINDOW = "Occupancy-Gird";

nav_msgs::OccupancyGrid Final_Grid;

ros::Subscriber sub_Lanedata_left;
ros::Subscriber sub_Lanedata_right;

ros::Publisher pub_Lanedata;

std::vector<std::vector<Point> > Lane_points_left;
std::vector<std::vector<Point> > Lane_points_right;

Mat src_left, src_right;
Mat roi, final_grid;

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


int main(int argc, char **argv)
{
	double begin,end;
	int iterations_dilate = 1;
	int ker_size_dilate = 2;
	Mat kernel_dilate;
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	ros::init(argc, argv, "Lane_Occupancy_Grid");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);

	image_transport::Publisher pub;
	
    
	image_transport::Subscriber sub_left = it.subscribe("/camera/image_raw1", 1, leftimage);
	image_transport::Subscriber sub_right = it.subscribe("/camera/image_raw3", 1, rightimage);

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	createTrackbar("iterations_dilate", WINDOW, &iterations_dilate, 50);
	createTrackbar("ker_size_dilate", WINDOW, &ker_size_dilate, 50);
	pub = it.advertise("/Interpolater", 1);

	ros::Rate loop_rate(15);

	while(ros::ok())
	{	

		//begin = ros::Time::now().toSec();
		final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
		ros::spinOnce();
		final_grid = final_grid(Rect(150,0,200,400));
		//cvtColor(final_grid,final_grid,CV_GRAY2BGR);
		iterations_dilate = getTrackbarPos("iterations_dilate", WINDOW);
		ker_size_dilate = getTrackbarPos("ker_size_dilate",WINDOW)*2 + 1;
		kernel_dilate = getStructuringElement(MORPH_ELLIPSE, Size (ker_size_dilate,ker_size_dilate), Point(-1,-1));
		dilate(final_grid, final_grid, kernel_dilate,Point(-1,-1), iterations_dilate);
		findContours(final_grid, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();
		pub.publish(msg);
		imshow(WINDOW, final_grid);
		//Final_Grid.data.clear();
		final_grid.release();
		// end = ros::Time::now().toSec();
		// cout<< end - begin <<endl;
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");
}