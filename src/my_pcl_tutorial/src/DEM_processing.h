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
#include <opencv2/contrib/contrib.hpp>
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

int c = 0;              // Image storing variable

double min_x, max_x;    // Minimum and maximum values of lateral X in Bird's eye view
double min_z, max_z;    // Minimum and maximum values of longitudinal Z in Bird's eye view
double min_y, max_y;    // Minimum and maximum values of height to be considered for intensity scaling

Mat depth_elevation_map = Mat::zeros((max_z - min_z)*100, (max_x - min_x)*100, CV_8UC1);    // Depth elevation map in Bird's eye view(30 x 12 meters view)
Mat density_map = Mat::zeros((max_z - min_z)*20, (max_x - min_x)*20, CV_8UC1);              // To calculate pixel density in each 5x5 cm square cell of DEM
Mat DEM_compressed = Mat::zeros(960, 1280, CV_8UC1);                                        // Generating compressed version of DEM based on variation of point density
Mat compressed_space;                                                                       // Cropping out region of interest from DEM_compressed

double kc=6.0;          // 6 columns single grid. This scales down column space by 6 to generate compressed space
double k=0.0766518;     // This is empirically chosen based on how much variation we need along longitudinal direction
double Z0=1;            // Minimum value of Z in meters for compressed space
double f=1158.0;        // Focal length of the camera in pixels
int col_max, col_min, row_max, row_min;   // maximum and minimum row/column values to define boundaries of region of interest in compressed space
int Ix = 624            // Image center X-coordinate as published in camera_info

// To save different images with different names
string image_number;

/*
  -> Defining ranges of scene we want to cover
  -> Here 3 dimensional system we are using is right hand co-ordinate system.
  -> +ve X is in right direction
  -> +ve Z is in forward direction(in the direction of depth with respect to the camera)
  -> +ve Y is downward direction
*/
max_x = 6.0;
min_x = -6.0;
max_y = 1.8;
min_y = 1.2;
max_z = 30.0;
min_z = 0.0;


void compute_DEM(const pcl::PointCloud& cloud){

  for(size_t  i=0; i < cloud->points.size();++i)
  {
      if((cloud->points[i].x < max_x && cloud->points[i].x > min_x) && (cloud->points[i].y < max_y && cloud->points[i].y > min_y) && (cloud->points[i].z < max_z && cloud->points[i].z > min_z)){

        density_map.at<uchar>((600 - (int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%5))/5.0), int((int(100 * (cloud->points[i].x - min_x)) - int(100 * (cloud->points[i].x - min_x))%5)/5))++;
          for(int j = 0; j < 5; j++){
            //cout << "i value is:" << 300 - int((int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%10))/10.0) << " " << "j value is:" << int(int(100 * (cloud->points[i].x - min_x)) - int(100 * (cloud->points[i].x - min_x))%10)/10 << endl;
            //density_map.at<uchar>(600 - int((int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%5))/5.0), int(int(100 * (cloud->points[i].x - min_x)) - int(100 * (cloud->points[i].x - min_x))%5)/5)++;
            depth_elevation_map.at<uchar>(3000 - (int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%5) + j), int(100 * (cloud->points[i].x - min_x)) - int((int(100 * (cloud->points[i].x - min_x)))%5) + j) = (depth_elevation_map.at<uchar>(3000 - (int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%5) + j), int(100 * (cloud->points[i].x - min_x)) - int((int(100 * (cloud->points[i].x - min_x)))%5) + j) + int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y))))/2;


      }
    }
  }
}

void compute_compressedSpace(const pcl::PointCloud& cloud){
  for(size_t  i=0; i < cloud->points.size();++i)
    {

      if((cloud->points[i].x < max_x && cloud->points[i].x > min_x) && (cloud->points[i].y < max_y && cloud->points[i].y > min_y) && (cloud->points[i].z < max_z && cloud->points[i].z > Z0)){
      double X=cloud->points[i].x;
      double Y=cloud->points[i].y;
      double Z=cloud->points[i].z;

      int col = int((Ix - int((f*X)/(Z)))/6.0);
      if(col > col_max) col_max = col;
      if(col < col_min) col_min = col;
      //int col = 1;
      int row = int(log(Z/Z0)/log((1+k)));
      if(row > row_max) row_max = row;
      if(row < row_min) row_min = row;
      DEM_compressed.at<uchar>(row,col) = max(int(DEM_compressed.at<uchar>(row,col)), int(255.0 * ((max_y - Y)/(max_y - min_y))));

    }

  }
}

void rectifyDEM(){
  for(int j = 0; j < density_map.cols; j++){
    for(int i = 0; i < density_map.rows; i++){
          if(density_map.at<uchar>(i, j) != 0){
              float propagation_range;
              float scaled_density = (180.0*density_map.at<uchar>(i, j))/255.0 + 20.0;
              float proportionality_constant = 600.0; // Derived analytically
              propagation_range = proportionality_constant/scaled_density;
              for(int k = i+1; k < (i + int(propagation_range)); k++){
                if(density_map.at<uchar>(k, j) == 0){
                  for(int a = 0; a < 5; a++){
                    if(int(depth_elevation_map.at<uchar>(5*k + a, 5*j + a)) != 0) break;
                    depth_elevation_map.at<uchar>(5*k + a, 5*j + a) = int(depth_elevation_map.at<uchar>(5*i, 5*j));
                }
              }
            }
        }
    }
  }

}


void compressedSpacetoDEM(){
  for(int  j=0; j < depth_elevation_map.cols; j++)
  {
      for(int i=100; i < depth_elevation_map.rows; i++){
        if(int(depth_elevation_map.at<uchar>(i, j)) == 0){
        double Z = i/100.0;
        double X = j/100.0 +  min_x;
        if((X < max_x && X > min_x) && (Z < max_z && Z > 1.0)){
        int row = int(log(Z/Z0)/log((1+k)));
        int col = int((624 - int((f*X)/Z))/6.0);
        if((row < DEM_compressed.rows && row > 0) && (col < DEM_compressed.cols && col > 0)) depth_elevation_map.at<uchar>(3000 - i, j) = int(DEM_compressed.at<uchar>(row,col));
      }
      }
    }
  }

}
