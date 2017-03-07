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

int sep = 50;
int interpol_height = 30;
int epsilon = 6;
int angle = 20;
double begin,end;
Mat src, src_gray;
Mat src_rgb;
Mat roi, final_grid;
//Mat LeftROI;
void leftimage(const sensor_msgs::ImageConstPtr& original_image)
{
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

	Lane_points_left.clear();//comment one of these

	Mat LeftROI;//LeftROI = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
	roi.copyTo(LeftROI);

	if (!final_grid.data)//why
	final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);

	final_grid = final_grid + LeftROI;
    //how about
    //final_grid = final_grid + roi;

	Lane_points_left.clear();

	waitKey(1);
}

void rightimage(const sensor_msgs::ImageConstPtr& original_image)
{
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

	Lane_points_right.clear();//comment one of these

	Mat RightROI;
	roi.copyTo(RightROI);

	if (!final_grid.data)//why
	final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);

	final_grid = final_grid + RightROI;


	Lane_points_right.clear();

	waitKey(1);
}

void extend(std::vector<std::vector<Point> > &Lane_points)
{
    for (int i = 0; i < Lane_points.size(); ++i)
    {
        int max_j = 0;

        for (int j = 0; j < Lane_points[i].size(); ++j)
        {
            if (Lane_points[i][max_j].y < Lane_points[i][j].y)
            {
                max_j = j;
            }
        }

        int k = 0;

        for (int j = 0; j < Lane_points[i].size(); ++j)
        {
            int diffx = Lane_points[i][max_j].x - Lane_points[i][j].x;
            int diffy = Lane_points[i][max_j].y - Lane_points[i][j].y;

            float dist = sqrt(diffx*diffx + diffy*diffy);

            if (dist > sep)
            {
                k = j;
            }

            if ((j == Lane_points[i].size() - 1) && (dist < sep) && (k == -1))
            {
                k = -1;
            }
        }

        if ((k != -1) && (Lane_points[i][max_j].y > occ_grid_heightr - interpol_height))
        {
            Point pt;
            float m, c;

            if ((float)Lane_points[i][k].y != (float)Lane_points[i][max_j].y)
            {
                m = ((float)Lane_points[i][k].x - (float)Lane_points[i][max_j].x)/((float)Lane_points[i][k].y - (float)Lane_points[i][max_j].y);
                c = (float)Lane_points[i][max_j].x - m*(float)Lane_points[i][max_j].y;

                pt.y = occ_grid_heightr - 1;
                pt.x = (int)(m*(float)pt.y + c);

                if ((Lane_points[i][max_j].x < image_width/2) && (Lane_points[i][k].x < image_width/2) && (pt.x < image_width/2) && (abs(m) < tan(angle*CV_PI/180.0)))
                line(src, Lane_points[i][max_j], pt, Scalar(255, 255, 255));

                else if ((Lane_points[i][max_j].x > image_width/2) && (Lane_points[i][k].x > image_width/2) && (pt.x > image_width/2) && (abs(m) < tan(angle*CV_PI/180.0)))
                line(src, Lane_points[i][max_j], pt, Scalar(255, 255, 255));
                    
                else
                {
                    pt.y = occ_grid_heightr - 1;
                    pt.x = Lane_points[i][max_j].x;

                    line(src, Lane_points[i][max_j], pt, Scalar(255, 255, 255));
                }

                circle(src_rgb, Lane_points[i][k], 6, Scalar(255, 255, 0));
                circle(src_rgb, Lane_points[i][max_j], 6, Scalar(255, 255, 0));
                circle(src_rgb, pt, 6, Scalar(255, 255, 0));
                line(src_rgb, Lane_points[i][k+1], Lane_points[i][1], Scalar(255, 255, 0));

                // cout<<m<<endl;
            }

            else
            {
                pt.y = occ_grid_heightr - 1;
                pt.x = Lane_points[i][max_j].x;

                line(src, Lane_points[i][max_j], pt, Scalar(255, 255, 255));
            }
        }

        else if ( Lane_points[i][max_j].y > occ_grid_heightr - interpol_height)
        {
            Point pt;

            pt.y = occ_grid_heightr - 1;
            pt.x = Lane_points[i][max_j].x;

            line(src, Lane_points[i][max_j], pt, Scalar(255, 255, 255));
        }
    }
}

void interpolate(int, void*)
{
    src = final_grid;
    std::vector<std::vector<Point> > contours;
    std::vector<std::vector<Point> > Lane_points;
    std::vector<Vec4i> hierarchy;

    cvtColor(src, src_gray, CV_BGR2GRAY);// no need to convert src is already in Gray
    findContours(src_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    src_rgb = Mat::zeros(Size(occ_grid_widthr, occ_grid_heightr), CV_8UC3);

    for (int i = 0; i < contours.size(); ++i)
    {
        std::vector<Point> vec;

        approxPolyDP(Mat(contours[i]), vec, (double)epsilon, true);

        Lane_points.push_back(vec);
    }

    extend(Lane_points);

    for (int i = 0; i < Lane_points.size(); ++i)
    {
        circle(src_rgb, Lane_points[i][0], 4, Scalar(255, 255, 0));
        circle(src_rgb, Lane_points[i][Lane_points[i].size() - 1], 4, Scalar(255, 255, 255));

        for (int j = 0; j < Lane_points[i].size() - 1; ++j)
        {
            line(src_rgb, Lane_points[i][j], Lane_points[i][j+1], Scalar(255, 255, 255), 1);
        }
    }
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

    cvtColor(src,src_gray, CV_BGR2GRAY);

    for (int i = 0; i < src_gray.rows; ++i)
    {
        for (int j = 0; j < src_gray.cols; ++j)
        {
            if ( src_gray.at<uchar>(i,j) > 0)
            {
                //cout<<"r";
                Final_Grid.data.push_back(2);

            }
            else
                Final_Grid.data.push_back(src_gray.at<uchar>(i,j));
        }
    }   

    pub_Lanedata.publish(Final_Grid);
    waitKey(1);
    imshow(WINDOW, src);
    imshow("src_rgb", src_rgb);

    Final_Grid.data.clear();

    Lane_points.clear();
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

	pub_Lanedata = nh.advertise<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid", 1);

	ros::Rate loop_rate(15);

	while(ros::ok())
	{
		//begin = ros::Time::now().toSec();
		final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
		ros::spinOnce();
		final_grid = final_grid(Rect(150,0,200,400));
		cvtColor(final_grid,final_grid,CV_GRAY2BGR);//no need to convert to BGR.....remove
		//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();
		//pub.publish(msg);
		interpolate(0,0);
		//imshow(WINDOW, Final_Grid);
		//Final_Grid.data.clear();
		final_grid.release();
		// end = ros::Time::now().toSec();
		// cout<< end - begin <<endl;
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");
}