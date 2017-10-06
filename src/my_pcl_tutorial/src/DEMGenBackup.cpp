#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/filters/filter.h>
#include <sstream>
#include <string.h>
#include <math.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace pcl;
using namespace image_transport;
using namespace message_filters;

int a = 0;
int count_valid_y = 0;
string save_depth_grey_path = "/home/dhaivat666/test/";
string save_depth_heatmap_path = "/media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/src/my_pcl_tutorial/Heat_images/";
string image_number;
image_transport::Publisher pub ;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	cout << "Old a " << a << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg (*input, *cloud_in);
    int height=cloud_in->height;
    int width=cloud_in->width;
    double  max_x=0,max_y=0,max_z=0,min_x=0,min_y=0,min_z=0;

    // cout<<"I am here "<<endl;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in,*cloud, indices); // Remove not a number values. It appears when there is no correpondance between points in both images because of occulsion.
    cout << cloud_in->points.size() <<" "<< cloud->points.size() << endl;
    cout << "Point cloud size: " << cloud->points.size() << endl;
    //for(i=0;i<indices.size();i++)
    //cout<<i<<" ";
    /*for(size_t i=0;i<cloud->points.size();++i)
    {

          if(cloud->points[i].x > max_x)
            max_x=cloud->points[i].x;

          if(cloud->points[i].x < min_x)
            min_x=cloud->points[i].x;


          if(cloud->points[i].y > max_y)
            max_y=cloud->points[i].y;

          if(cloud->points[i].y < min_y)
            min_y=cloud->points[i].y;


          if(cloud->points[i].z > max_z)

          if(cloud->points[i].z < min_z)
            max_z=cloud->points[i].z;
            min_z=cloud->points[i].z;

    } */

    // Defining ranges of scene we want to cover
    max_x = 6.0;
    min_x = -6.0;
    max_y = 2.0;
    min_y = -2.0;
    max_z = 20.0;
    min_z = 0.0;

    cout << "Max_x: " << max_x << "  " << "Min_x: " << min_x << endl;
    cout << "Max_y: " << max_y << "  " << "Min_y: " << min_y << endl;
    cout << "Max_z: " << max_z << "  " << "Min_z: " << min_z << endl;

    //cout<<max_x<<" "<<max_y<<" "<<max_z<<" "<<min_x<<" "<<min_y<<" "<<min_z<<endl;
    Mat depth_elevation_map = Mat::zeros((max_z - min_z)*100, (max_x - min_x)*100, CV_8UC1);
 //    Mat heat_map_depth = depth_elevation_map.clone();
 /*Mat depth_elevation_map = Mat::zeros(1280, 960, CV_8UC1);

    Mat inpaint = depth_elevation_map.clone();
    Mat intermediate = depth_elevation_map.clone();*/
    //Mat img = Mat::zeros(int(2*(max_z))+1,int(2*(max_x))+1, CV_8UC1);
  for(size_t  i=0; i < cloud->points.size();++i)
    {
        if((cloud->points[i].x < max_x && cloud->points[i].x > min_x) && (cloud->points[i].y < max_y && cloud->points[i].y > min_y) && (cloud->points[i].z < max_z && cloud->points[i].z > min_z)){

        //cout << "Y value is: " << int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y))) << endl;
          //depth_elevation_map.at<uchar>(int(1280 * ((max_z - cloud->points[i].z)/(max_z - min_z))), int(960.0 * ((cloud->points[i].x-min_x)/(max_x - min_x)))) =  int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y)));
	        // if(cloud->points[i].y > 0 && cloud->points[i].y < 2.5) count_valid_y++;
		//depth_elevation_map.at<uchar>(int(100 * (cloud->points[i].z - min_z)), int(100 * (cloud->points[i].x - min_x))) = int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y)));
         if(depth_elevation_map.at<uchar>(100*max_z - int(100 * cloud->points[i].z), int(100 * (cloud->points[i].x - min_x))) < int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y)))){
              for(int j = 1; j <= 10; j++){
                depth_elevation_map.at<uchar>(100*max_z - int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%10) + j, int(100 * (cloud->points[i].x - min_x)) - int((int(100 * (cloud->points[i].x - min_x)))%10) + j) = int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y)));
              }

            }


        }}

        // cout << "Z:" << int(1280 * ((cloud->points[i].z-min_z)/(max_z - min_z))) << "  " << "X:" << int(960.0 * ((cloud->points[i].x-min_x)/(max_x - min_x))) << "  " << "Y:" << int(255.0 * ((cloud->points[i].y - min_y)/(max_y - min_y))) << endl;
    //}

    // Changing colormap
   // applyColorMap(depth_elevation_map, heat_map_depth, COLORMAP_JET);
    stringstream out;
    out << a;
    image_number = out.str();
   save_depth_grey_path = save_depth_grey_path + image_number + ".ppm";
	 std::cout << save_depth_grey_path << std::endl;
// save_depth_heatmap_path = save_depth_heatmap_path + image_number + ".jpg";

   imwrite(save_depth_grey_path, depth_elevation_map);
 // imwrite(save_depth_heatmap_path, heat_map_depth);
    a++;
    save_depth_grey_path = "/home/dhaivat666/test/";
 //   save_depth_heatmap_path = "/media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/src/my_pcl_tutorial/Heat_images/";
  //  imshow("Depth elevation map", depth_elevation_map);
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_elevation_map).toImageMsg();
    //pub.publish(msg);
    cout << "New a " << a << endl;
  }

/*
void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  Mat left_image = cv_bridge::toCvShare(msg, "bgr8")->image;
  save_left_image_path = "../my_pcl_tutorial/Left_images/" + image_number + ".jpg";
  imwrite(save_depth_grey_path, left_image);
  save_left_image_path = "../my_pcl_tutorial/Left_images/";
  j++;

}
*/

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // Create a ROS subscriber for the input point cloud
 ros::Subscriber sub = nh.subscribe ("/camera/points2", 1, cloud_cb);
//  image_transport::Subscriber left_image = it.subscribe("/near_stereo/left/image_raw", 10, image_callback);
  namedWindow("Depth elevation map", CV_WINDOW_AUTOSIZE);
  // Create a ROS publisher for the output point cloud
  //pub = it.advertise("/DEM_IMAGE", 1);
  cout << "1" << endl;

  // Spin
  ros::spin ();
}
