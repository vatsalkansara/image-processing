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
	ros::init(argc, argv, "Lane_Occupancy_Grid");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Publisher pub;
	
    
	image_transport::Subscriber sub_left = it.subscribe("/camera/image_raw1", 1, leftimage);
	image_transport::Subscriber sub_right = it.subscribe("/camera/image_raw3", 1, rightimage);

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

	pub = it.advertise("/Interpolater", 1);

	ros::Rate loop_rate(15);

	while(ros::ok())
	{

		final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
		ros::spinOnce();
		final_grid = final_grid(Rect(150,0,200,400));
		cvtColor(final_grid,final_grid,CV_GRAY2BGR);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();
		pub.publish(msg);
		imshow(WINDOW, final_grid);
		//Final_Grid.data.clear();
		final_grid.release();
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");
}