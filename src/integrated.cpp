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

//mapping global variables
const float res = 0.25;          //angular resolution of lidar in degrees
const int num_values = 1080;     //no. of rays emitted by the lidar

const int radius_in_cm = 60;     //radius around on object to be in safe limit for path-planning
const int lane_in_cm = 30;       //radius around lane for path planning
const int side_in_cm = 30;
const int base_lidar_dist = 50;   //distance from vehicle centre to lidar in cm(only in y-direction)

const int cam_lidar_dist = 50;    //distance from camera to lidar in cm
const int width = 500;//map size
const int height = 250;
const int range = 200;//10m in cells
const int l_width = 200;//lane map size 
const int l_height = 400;

float map_res = 0.05;//map resolution(m/cell)
float radius_in_cells;
float minor_axis_in_cells = (lane_in_cm*0.01)/map_res;//elliptically blowing lanes
float major_axis_in_cells = minor_axis_in_cells + 3;
float base_lidar_dist_in_cells = (base_lidar_dist*0.01)/map_res;
float base_cam_dist_in_cells = ((cam_lidar_dist - base_lidar_dist)*0.01)/map_res;
//mapping global variables end

namespace enc = sensor_msgs::image_encodings;
string WINDOW = "Occupancy-Gird";

nav_msgs::OccupancyGrid grid_map,blown_grid_map;

ros::Subscriber sub_Lanedata_left;
ros::Subscriber sub_Lanedata_right;

//ros::Publisher pub_Lanedata;
ros::Publisher map_pub,blown_map_pub;

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
        //  Lanepoints.push_back(cv::Point2f( (float)contours[i][j].x, (float)contours[i][j].y ));
        // }
    }
 //    if (Lanepoints.size() > 0)
 //     {
 //       compactness = kmeans(Lanepoints, 2, labels, TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 2, KMEANS_RANDOM_CENTERS);
 //       cout<<compactness<<endl;
  // }

  //Lanepoints.clear();
  // Final_Grid.info.map_load_time = ros::Time::now();
 //    Final_Grid.header.frame_id = "lane";
 //    Final_Grid.info.resolution = (float)map_width/(100*(float)occ_grid_widthr);
 //    Final_Grid.info.width = 200;
 //    Final_Grid.info.height = 400;

 //    Final_Grid.info.origin.position.x = 0;
 //    Final_Grid.info.origin.position.y = 0;
 //    Final_Grid.info.origin.position.z = 0;

 //    Final_Grid.info.origin.orientation.x = 0;
 //    Final_Grid.info.origin.orientation.y = 0;
 //    Final_Grid.info.origin.orientation.z = 0;
 //    Final_Grid.info.origin.orientation.w = 1;

 //    for (int i = 0; i < dst.rows; ++i)
 //    {
 //        for (int j = 0; j < dst.cols; ++j)
 //        {
 //            if ( dst.at<uchar>(i,j) > 0)
 //            {
 //                //cout<<"r";
 //                Final_Grid.data.push_back(2);

 //            }
 //            else
 //                Final_Grid.data.push_back(dst.at<uchar>(i,j));
 //        }
 //    }
 //    pub_Lanedata.publish(Final_Grid);
 //    Final_Grid.data.clear();

}
//mapping functions
pair<float,float> convToCart(int i,float r) //convert from polar to cartesian co-ordinates
{
   float ang = i*0.25*(3.14/180);
   float x = r*cos(ang);
   float y = -1*r*sin(ang);
   return make_pair(x,y);
}
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   //cout<<base_lidar_dist_in_cells<<endl;
   int i,j;

   //nav_msgs::OccupancyGrid grid_map;
   grid_map.header.stamp = ros::Time::now();
   grid_map.header.frame_id = "/lidar";
   grid_map.info.resolution = map_res;
   grid_map.info.origin.position.x = 0.0;
   grid_map.info.origin.position.y = 0.0;
   grid_map.info.origin.position.z = 0.0;
   grid_map.info.origin.orientation.x = 0.0;
   grid_map.info.origin.orientation.y = 0.0;
   grid_map.info.origin.orientation.z = 0.0;
   grid_map.info.origin.orientation.w = 1.0;
   grid_map.info.width = width;
   grid_map.info.height = height;
   grid_map.info.map_load_time = ros::Time::now();

   //nav_msgs::OccupancyGrid blown_grid_map;
   blown_grid_map.header.stamp = ros::Time::now();
   blown_grid_map.header.frame_id = "/lidar";
   blown_grid_map.info.resolution = map_res;
   blown_grid_map.info.origin.position.x = 0.0;
   blown_grid_map.info.origin.position.y = 0.0;
   blown_grid_map.info.origin.position.z = 0.0;
   blown_grid_map.info.origin.orientation.x = 0.0;
   blown_grid_map.info.origin.orientation.y = 0.0;
   blown_grid_map.info.origin.orientation.z = 0.0;
   blown_grid_map.info.origin.orientation.w = 1.0;
   blown_grid_map.info.width = width;
   blown_grid_map.info.height = height;
   blown_grid_map.info.map_load_time = ros::Time::now();
   
   vector< pair<float,float> > cartesian(num_values,make_pair(0,0));
   for(i=0;i<num_values;i++)
   {
      cartesian[i] = convToCart((i-num_values/2),msg->ranges[i]);
      cartesian[i].first += base_lidar_dist*0.01;
   }
   //float min=20;
   /*for(i=0;i<num_values;i++)
   {
      if(msg->ranges[i]<min)
      {min=msg->ranges[i];}
      //if(msg->ranges[i]==0.0)
      //{cout<<i;}
   }*/
   //cout<<"min="<<min<<",straight="<<msg->ranges[540]<<endl;
   //int def=round(cartesian[540].first/map_res);
   //cout<<def;
   //std_msgs::Int8MultiArray map;
   for(i=0;i<height;i++)//initialising map
   {
      for(j=0;j<width;j++)
      {
         grid_map.data.push_back(0);
         blown_grid_map.data.push_back(0);
      }
   }
   //cout<<"hi"<<endl;
   for(i=0;i<num_values;i++)
   {
      int x_coord = round(cartesian[i].first/map_res);
      int y_coord = round(cartesian[i].second/map_res);
      /*if(x_coord==0 && y_coord==0)
      {
        cout<<i<<" "<<msg->ranges[i];
      }*/
      if(x_coord > 0 && x_coord <= range)//limiting obstacle detection to only 10m
      {
         if(y_coord <= range && y_coord >= -range)
         {
            if(!(x_coord==base_lidar_dist_in_cells && y_coord==0))
            grid_map.data[x_coord*width+y_coord+width/2] = 1;
         } 
      }
   }
}

void add_lane()
{
  for(int i=0;i<l_height;i++)//adding lane data 
   {
      for(int j=0;j<l_width;j++)
      {
         int i1=l_height-1-i-base_cam_dist_in_cells;
         if(i1>=0)
         { 
            if(dst.at<uchar>(i1,j) > 0)
            grid_map.data[i*width+j+width/2-l_width/2] = 2;
         }
      }
   }
}
void blowmap()
{
  for(int i=0;i<height;i++)//creating blown map
   {
      for(int j=0;j<width;j++)
      {
         if(grid_map.data[i*width+j]==1)//blowing obstacle data
         {
            if(i<base_lidar_dist_in_cells)
            radius_in_cells = (side_in_cm*0.01)/map_res;
            else
            radius_in_cells = (radius_in_cm*0.01)/map_res;

            for(int k=-1*radius_in_cells;k<=radius_in_cells;k++)
            {
               for(int l=-1*radius_in_cells;l<=radius_in_cells;l++)
               {
                  if((k*k + l*l) < radius_in_cells*radius_in_cells)
                  {
                     int x = i + k;
                     int y = j + l - width/2;
                     if(x >= 0 && x <= range)
                     {
                        if(y <= range && y >= -range)
                        {
                           blown_grid_map.data[x*width+y+width/2] = 1;
                        }
                     }
                  }
               }
            }
         }
         
         if(grid_map.data[i*width+j]==2)//blowing lane data
         {
            for(int k=-1*major_axis_in_cells;k<=major_axis_in_cells;k++)
            {
               for(int l=-1*minor_axis_in_cells;l<=minor_axis_in_cells;l++)
               {
                  if(((minor_axis_in_cells*minor_axis_in_cells)*(k*k) + (major_axis_in_cells*major_axis_in_cells)*(l*l)) < (minor_axis_in_cells*minor_axis_in_cells)*(major_axis_in_cells*major_axis_in_cells))//inside ellipse condition
                  {
                     int x = i + k;
                     int y = j + l - width/2;
                     if(x >= 0 && x <= range)
                     {
                        if(y <= range && y >= -range)
                        {
                           blown_grid_map.data[x*width+y+width/2] = 2;
                        }
                     }
                  }
               }
            }
         }

      }
   }
   //map_pub.publish(grid_map);
   blown_map_pub.publish(blown_grid_map);
   Mat local_map = Mat::zeros(height,width,CV_8UC1);
   Mat blown_local_map = Mat::zeros(height,width,CV_8UC1);
   for(int i=0;i<height;i++)
   {
     for(int j=0;j<width;j++)
     {
       int i_= height - i;
       local_map.at<uchar>(i,j) = grid_map.data[i_*width+j]*0.5*255;
       blown_local_map.at<uchar>(i,j) = blown_grid_map.data[i_*width+j]*0.5*255;
     }
   }
   imshow("local_map",local_map);
   imshow("blown_local_map",blown_local_map);
   waitKey(1);
   grid_map.data.clear();
   blown_grid_map.data.clear();
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
  namedWindow("local_map",CV_WINDOW_NORMAL);
    namedWindow("blown_local_map",CV_WINDOW_NORMAL);
  createTrackbar("iterations_dilate", WINDOW, &iterations_dilate, 10);
  createTrackbar("ker_size_dilate", WINDOW, &ker_size_dilate, 10);
  createTrackbar("area", WINDOW, &area, 200);
  //pub = it.advertise("/Lane_Occupancy_Grid", 1);
  //map_pub=n2.advertise<nav_msgs::OccupancyGrid>("scan/local_map",1);
    blown_map_pub=nh.advertise<nav_msgs::OccupancyGrid>("scan/blown_local_map",1);
  //pub_Lanedata = nh.advertise<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid", 1);
  ros::Rate loop_rate(10);

  while(ros::ok())
  { 

    //begin = ros::Time::now().toSec();
    final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
    ros::spinOnce();
    final_grid = final_grid(Rect(150,0,200,400));
    //cvtColor(final_grid,final_grid,CV_GRAY2BGR);
    endprocessing();
    add_lane();
    blowmap();
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