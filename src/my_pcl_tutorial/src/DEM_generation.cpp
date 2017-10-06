// Library files
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

// Namespaces
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace pcl;
using namespace image_transport;
using namespace message_filters;

int c = 0;    // Image count. It increases each time new point clound corresponding to new frame come

// Defining paths for different data to be saved
string save_DEM = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Depth_elevation_images/";
string save_heat_DEM = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Heat_map_depth_elevation_images/";
string save_CS = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Compressed_space/";
string save_heat_CS = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Heat_compressed_space/";
string save_Ex_DEM = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Expanded_images/";

image_transport::Publisher pub;

// Input point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

// Processed point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

// Scene range variables
double max_x = 6.0;
double min_x = -6.0;
double max_y = -0.5;
double min_y = -1.5;
double max_z = 30.0;
double min_z = 1.0;

// To remove NaN values
std::vector<int> indices;

Mat depth_elevation_map = Mat::zeros((max_z - min_z)*100, (max_x - min_x)*100, CV_8UC1);   // Depth elevation map for 30 m x 12 m is saved.
Mat heat_depth_elevation_map = depth_elevation_map.clone();                                // Heat map of depth elevation map
Mat pixel_count = Mat::zeros((max_z - min_z)*20, (max_x - min_x)*20, CV_8UC1);             // Pixel density in each 5 cm x 5 cm cell of DEM
Mat DEM_compressed = Mat::zeros(960, 1280, CV_8UC1);                                       // Intermediate image to generate compressed space
Mat expanded_image = depth_elevation_map.clone();                                          // Expanding compressed space
Mat compressed_space;                                                                      // Compressed space from point cloud data
Mat heat_compressed_space;                                                                 // Heat map of a compressed space

double kc=6.0;      // 6 column single grid
double k=0.0766518; // Empirically chosen value used in generation of a compressed space
double Z0=1;        // From where compressed space starts
double f=1158.0;    // Focal length of a camera in pixels

// Finiding out range of compressed space in DEM_compressed
int col_max = 1;
int col_min = 10;
int row_max = 1;
int row_min = 30;

/*
  This function generates compressed space from a point cloud. Its purpose is to generate a small space to counteract issue of point cloud sparsity along far depths.
  Method implemented in this is explained in "Obstacle Detection Based on Dense Stereovision for Urban ACC Systems."
 */

void gen_compressed_space() {

      for(size_t  i=0; i < cloud->points.size();++i)
        {
          // Only proceed forward if point falls in a range of interest
          if((cloud->points[i].x < max_x && cloud->points[i].x > min_x) && (cloud->points[i].y < max_y && cloud->points[i].y > min_y) && (cloud->points[i].z < max_z && cloud->points[i].z > Z0)){
          double X=cloud->points[i].x;
          double Y=cloud->points[i].y;
          double Z=cloud->points[i].z;

          int col = int((624 - int((f*X)/(Z)))/6.0);  // Estimating column corresponding to point X,Y,Z in compressed space

          // Estimating range of column values in comrpessed space
          if(col > col_max) col_max = col;
          if(col < col_min) col_min = col;

          int row = int(log(Z/Z0)/log((1+k)));    // Estimating row corresponding to the point X,Y,Z in compressed space

          // Estimating range of row values in compressed space
          if(row > row_max) row_max = row;
          if(row < row_min) row_min = row;
          DEM_compressed.at<uchar>(row,col) = (int(DEM_compressed.at<uchar>(row,col)) + int(255.0 * ((max_y - Y)/(max_y - min_y))))/2;

        }
      }

      // Defining region of interest based on row and column values
      Rect myROI(0, 0, col_max - (col_max%10) + 5, row_max - (row_max%10) + 5);

      // Copying region of interest in compressed space
      Mat(DEM_compressed, myROI).copyTo(compressed_space);

      // Generating heat map of a compressed space
//      applyColorMap(compressed_space, heat_compressed_space, COLORMAP_JET);

}

/*
    This functions generates depth elevation map from a received point cloud.
 */
void gen_DEM(){

      for(size_t  i=0; i < cloud->points.size();++i)
        {
          // Only proceed forward if point falls in a range of interest
          if((cloud->points[i].x < max_x && cloud->points[i].x > min_x) && (cloud->points[i].y < max_y && cloud->points[i].y > min_y) && (cloud->points[i].z < max_z && cloud->points[i].z > min_z)){

            // Increasing pixel_count value each time point falls in a particular cell
            pixel_count.at<uchar>((600 - (int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%5))/5.0), int((int(100 * (cloud->points[i].x - min_x)) - int(100 * (cloud->points[i].x - min_x))%5)/5))++;

            //  if(depth_elevation_map.at<uchar>(3000 - int(100 * cloud->points[i].z), int(100 * (cloud->points[i].x - min_x))) < int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y)))){
            depth_elevation_map.at<uchar>(3000 - int(100 * cloud->points[i].z), int(100 * (cloud->points[i].x - min_x))) = int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y)));
                // Here, 1 pixel corresponds to 1 CM of an actual scene, so we process entire 5 cm x 5 cm cell where a point of pointcloud falls
                /*for(int j = 0; j < 5; j++){

                  depth_elevation_map.at<uchar>(3000 - (int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%5) + j), int(100 * (cloud->points[i].x - min_x)) - int((int(100 * (cloud->points[i].x - min_x)))%5) + j) = (depth_elevation_map.at<uchar>(3000 - (int(100 * cloud->points[i].z) - int((int(100 * cloud->points[i].z))%5) + j), int(100 * (cloud->points[i].x - min_x)) - int((int(100 * (cloud->points[i].x - min_x)))%5) + j) + int(255.0 * ((max_y - cloud->points[i].y)/(max_y - min_y))))/2;

                }*/

            }
          }
        }


/*
    This function fills invalid cells of generated DEM by propograting heights along the columns based in point cloud density in a particular valid cell
 */
void rectifyDEM(){
    // Traversing along pixel_count image
      for(int j = 0; j < pixel_count.cols; j++){
        for(int i = 0; i < pixel_count.rows; i++){
            // Only change pixel values which are initially zero, this will avoid overwriting
            if(pixel_count.at<uchar>(i, j) != 0){
                float propagation_range;                                                  // Defining propagation range using inverse relation between propagation_range and point cloud density in a valid cell
                float scaled_density = (80.0*pixel_count.at<uchar>(i, j))/255.0 + 20.0;  // Scaling point cloud density for a valid proportinality
                float proportionality_constant = 300.0;                                   // Derived analytically
                propagation_range = proportionality_constant/scaled_density;

                // Propagating along columns
                for(int k = j+1; k < (j + int(propagation_range)); k++){
                  // Propagating only if value of the pixel is zero, meaning there is no 3D point in that particular cell.
                  if(pixel_count.at<uchar>(i, k) == 0){
                    // Filling 5 cm x 5 cm cell
                    for(int a = 0; a < 5; a++){
                      if(int(depth_elevation_map.at<uchar>(5*i + a, 5*k + a)) != 0) break;
                      depth_elevation_map.at<uchar>(5*i + a, 5*k + a) = int(depth_elevation_map.at<uchar>(5*i, 5*j));
                    }
                  }
                }
              }
            }
          }
          // generating heat map
  //        applyColorMap(depth_elevation_map, heat_depth_elevation_map, COLORMAP_JET);
        }

/*
  Traversing through the expanded space and replacing initialized value with a corresponding value of the compressed space
 */
void expandCompressedSpace(){
  for(int  j=0; j < expanded_image.cols; j++)
  {
      for(int i=100; i < expanded_image.rows; i++){
        if(int(expanded_image.at<uchar>(i, j)) == 0){
        double Z = i/100.0;
        double X = j/100.0 +  min_x;
        if((X < max_x && X > min_x) && (Z < max_z && Z > 1.0)){
        int row = int(log(Z/Z0)/log((1+k)));
        int col = int((624 - int((f*X)/Z))/6.0);
        if((row < DEM_compressed.rows && row > 0) && (col < DEM_compressed.cols && col > 0)) expanded_image.at<uchar>(3000 - i, j) = int(DEM_compressed.at<uchar>(row,col));
        }
      }
    }
  }
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

    pcl::fromROSMsg (*input, *cloud_in);
    int height=cloud_in->height;
    int width=cloud_in->width;

    string image_number;              // To convert image count to string
    stringstream out;                 // Intermediate variable to convert integer to string
    out << c;
    image_number = out.str();

    cout << c << endl;
    pcl::removeNaNFromPointCloud(*cloud_in,*cloud, indices); // Remove not a number values. It appears when there is no correpondance between points in both images because of occulsion.
    //gen_compressed_space();   // generate compressed DEM
    //gen_DEM();                // Generate normal DEM
    //rectifyDEM();             // It propagates height along the Column of generated DEM to maintain connectivity
    //expandCompressedSpace();  // Expands compressed space DEM

    save_DEM = save_DEM + image_number + ".jpg";              //  Path where Depth elevation map is saved
    save_heat_DEM = save_heat_DEM  + image_number + ".jpg";   //  Path to save heat map of a Depth elevation map
    save_CS = save_CS + image_number + ".jpg";                //  Path to save compressed space
    save_heat_CS = save_heat_CS  + image_number + ".jpg";     //  Path to save heat map of a compressed space
    save_Ex_DEM = save_Ex_DEM  + image_number + ".jpg";       //  Path to save expanded DEM
    /*
    imwrite(save_DEM, depth_elevation_map);
    imwrite(save_heat_DEM, heat_depth_elevation_map);
    imwrite(save_CS, compressed_space);
    imwrite(save_heat_CS, heat_compressed_space);
    imwrite(save_Ex_DEM, expanded_image);
    c++;
    */
    save_DEM = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Depth_elevation_images/";
    save_heat_DEM = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Heat_map_depth_elevation_images/";
    save_CS = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Compressed_space/";
    save_heat_CS = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Heat_compressed_space/";
    save_Ex_DEM = "/home/dhaivat666/catkin_ws_pointcloud/src/my_pcl_tutorial/Expanded_images/";

    // Redefining for next frame
    depth_elevation_map = Mat::zeros((max_z - min_z)*100, (max_x - min_x)*100, CV_8UC1);   // Depth elevation map for 30 m x 12 m is saved.
    heat_depth_elevation_map = depth_elevation_map.clone();                                // Heat map of depth elevation map
    pixel_count = Mat::zeros((max_z - min_z)*20, (max_x - min_x)*20, CV_8UC1);             // Pixel density in each 5 cm x 5 cm cell of DEM
    DEM_compressed = Mat::zeros(960, 1280, CV_8UC1);                                       // Intermediate image to generate compressed space
    expanded_image = depth_elevation_map.clone();                                          // Expanding compressed space
    compressed_space;                                                                      // Compressed space from point cloud data
    heat_compressed_space;                                                                 // Heat map of a compressed space
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_elevation_map).toImageMsg();
    //pub.publish(msg);
  }


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/points2", 1, cloud_cb);
  // image_transport::Subscriber left_image = it.subscribe("/near_stereo/left/image_raw", 10, image_callback);
  //namedWindow("Depth elevation map", CV_WINDOW_AUTOSIZE);
  // Create a ROS publisher for the output point cloud
  //pub = it.advertise("/DEM_IMAGE", 1);

  // Spin
  ros::spin ();
}
