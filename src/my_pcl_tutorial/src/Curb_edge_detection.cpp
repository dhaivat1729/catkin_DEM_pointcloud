#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/* Global declarations */

// Variables for smoothing and thresholding and erosion

// global variables for erosion
int erosion_type;
int erosion_elem = 0;
const int erosion_elem_max = 2;
int erosion_size = 1;
const int erosion_size_max = 15;

// global variables for dilation
int dilation_elem = 0;
const int dilation_elem_max = 2;
int dilation_type;
int dilation_size = 1;
const int dilation_size_max = 10;

/*
 *  Smoothing types:
 *      1. For median filter,   smoothing_type = 0
 *      2. For Gaussian filter, smoothing_type = 1
 */

int smoothing_type = 0;
int const no_of_smoothing_types = 1;

/*
 *  Thresholding types:
 *      1. For Otsu thresholding,       thresholding_type = 0
 *      2. For Thresholding,            thresholding_type = 1
 *      3. For Adaptive thresholding,   thresholding_type = 2
 *      4. For Canny edge detection,    thresholding_type = 3
 */

int thresholding_type = 2;
int const no_of_thresholding_types = 3;

// Defining median filter parameters
int blur_filter_parameter = 5;                       // Parameter(n) to set mask size, actual size of mask is 2*n + 3
int const blur_filter_parameter_max_value = 30;

// Defining gaussian filter parameters
int gaussian_filter_parameter = 5;                   // Parameter(n) to set mask size, actual size of mask is 2*n + 3
int const gaussian_filter_parameter_max_value = 10;

// Definint Otsu_thresholding parameters

// Defining thresholding parameters


// Defnining adaptive thresholding parameters
int block_size_parameter = 7;               // Block size
int const max_block_size_parameter = 30;
int mean_value = 2;                         // Mean value to be subtracted from adaptive thresholded image
int const max_mean_value = 10;

// Names of different trackbars
char* smoothing_name =  "Type: \n 0: Median filter \n 1: Gaussian filter ";
char* thresholding_name = "Type: \n 0: Otsu_thresholding \n 1: Thresholding \n 2: Adaptive thresholding \n 3: Canny edge detection";
char* median_smooth = "Mask width parameter: ";
char* gaussian_smooth = "Gaussian parameter: ";
char* block_name = "Block size parameter";
char* mean_value_trackbar_name = "Mean_value";
char* Erosion_element = "Provide erosion element: \n 0. MORPH_RECT \n 1. MORPH_CROSS \n 2. MORPH_ELLIPSE ";
char* Erosion_size = "Erosion size: ";
char* Dilation_element = "Choose dilation type: \n 0. MORPH_RECT \n 1. MORPH_CROSS \n 2. MORPH_ELLIPSE";
char* Dilation_size = "Vary dilation mask size: ";

// Different windows
char* main_window_name = "Image processing"; // Main window which will display different trackbars
char* Thresholded_image_display = "Thresholded image";    // Thresholded image will be displayed in this window
char* Smooth_image_display = "Smoothened image";          // Smooth image will be displayed here
char* Eroded_image_display = "Erored image"; // eroded image will be displayed here
char* Dilated_image_display = "Dilated_image"; // dilated image will be displayed here
char* Edge_image_display = "Edge_image"; // edge detected image using Scharr operator

/*
 *      src_Gray: Source grayscaled image.
 *      smooth_image: Smoothened image will be stored here.
 *      Thresholded_image: Thresholded image will be stored here.
 */
Mat src_Gray, smooth_image, thresholded_image, eroded_image, dilated_image, edge_image;

// This function will be called when any of the trackbar parameters will be changed
void Image_processing_function( int, void* ){

      if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
      else if(erosion_elem == 1){ erosion_type = MORPH_CROSS; }
      else if(erosion_elem == 2){ erosion_type = MORPH_ELLIPSE; }

      if ( dilation_elem == 0){ dilation_type = MORPH_RECT; }
      else if( dilation_elem == 1) { dilation_type = MORPH_CROSS; }
      else if ( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }


      // Defining structuring element
      Mat erosion_element = getStructuringElement(erosion_type, Size(2*erosion_size + 1, 2*erosion_size + 1), Point(-1, -1));
      Mat dilation_element = getStructuringElement(dilation_type, Size(2*dilation_size + 1, 2*dilation_size + 1), Point(-1, -1));

    switch(smoothing_type) {
        case 0:
            // smoothing_type 0 corresponds to median blurring
            dilate(src_Gray, dilated_image, dilation_element);
            erode(dilated_image, eroded_image, erosion_element);
            medianBlur(eroded_image, smooth_image, 2 * blur_filter_parameter + 3);
            switch(thresholding_type){
                case 0:
                    cout << "Otsu thresholding" << endl;
                    break;
                case 1:
                    cout << "Thresholding" << endl;
                    break;
                case 2:
                    adaptiveThreshold(smooth_image, thresholded_image, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 2*block_size_parameter + 3, mean_value);
                    Scharr(thresholded_image, edge_image, smooth_image.depth(), 1, 0, 5, 0, BORDER_DEFAULT);
                    convertScaleAbs(edge_image, edge_image);
                    // erode(thresholded_image, eroded_image, element);
                    //cout << "Adaptive thresholding" << endl;
                    break;
                case 3:
                    cout << "canny edgeDetection" << endl;
                    break;

            }
            break;

        case 1:
            // smoothing_type 1 corresponds to Gaussian blur
            dilate(src_Gray, dilated_image, dilation_element);
            erode(dilated_image, eroded_image, erosion_element);
            GaussianBlur(eroded_image, smooth_image, Size(2 * gaussian_filter_parameter + 1, 2 * gaussian_filter_parameter + 1), 0, 0);
            switch(thresholding_type){
                case 0:
                    cout << "Otsu thresholding1" << endl;
                    break;
                case 1:
                    cout << "Thresholding1" << endl;
                    break;
                case 2:
                    adaptiveThreshold(smooth_image, thresholded_image, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 2*block_size_parameter + 3, mean_value);
                    Scharr(thresholded_image, edge_image, smooth_image.depth(), 1, 0, 5, 0, BORDER_DEFAULT);
                    convertScaleAbs(edge_image, edge_image);
                    // erode(thresholded_image, eroded_image, element);
                    //cout << "Adaptive thresholding1" << endl;
                    break;
                case 3:
                    cout << "canny edgeDetection1" << endl;
                    break;
            }
            break;
    }

    imshow(main_window_name, src_Gray);                     // Main image will be displayed
    imshow(Smooth_image_display, smooth_image);             // smooth image will be displayed
    imshow(Thresholded_image_display, thresholded_image);   // thresholded image will be displayed
    imshow(Dilated_image_display, dilated_image);           // Dilated image will be displayed
    imshow(Eroded_image_display, eroded_image);             // eroded image will be displayed
    imshow(Edge_image_display, edge_image);

}


int main( int argc, char** argv ){

    src_Gray = imread( argv[1], 0 );                    // Input image
    /* negative of a source image
    Mat sub_mat = Mat::ones(src_Gray.size(), src_Gray.type())*255;
    subtract(sub_mat, src_Gray, src_Gray);
    */

    // Initializing windows
    namedWindow(main_window_name, CV_WINDOW_AUTOSIZE);
    namedWindow(Smooth_image_display, CV_WINDOW_AUTOSIZE);
    namedWindow(Thresholded_image_display, CV_WINDOW_AUTOSIZE);
    namedWindow(Eroded_image_display, CV_WINDOW_AUTOSIZE);
    namedWindow(Dilated_image_display, CV_WINDOW_AUTOSIZE);
    namedWindow(Edge_image_display, CV_WINDOW_AUTOSIZE);

    // Creating trackbars
    /*
    *  Parameters:
    *      1. Name of the trackbar
    *      2. Name of the window in which trackbar is to be displayed
    *      3. Passing reference of the parameteric value
    *      4. Maximum value
    *      5. Function which will be called when trackbar value is changed
    */
    createTrackbar(smoothing_name,
                    main_window_name, &smoothing_type,
                    no_of_smoothing_types, Image_processing_function );

    createTrackbar(thresholding_name,
                    main_window_name, &thresholding_type,
                    no_of_thresholding_types, Image_processing_function );

    createTrackbar(median_smooth,
                    main_window_name, &blur_filter_parameter,
                    blur_filter_parameter_max_value, Image_processing_function );

    createTrackbar(gaussian_smooth,
                    main_window_name, &gaussian_filter_parameter,
                    gaussian_filter_parameter_max_value, Image_processing_function );

    createTrackbar(block_name,
                    main_window_name, &block_size_parameter,
                    max_block_size_parameter, Image_processing_function );

    createTrackbar(mean_value_trackbar_name,
                    main_window_name, &mean_value,
                    max_mean_value, Image_processing_function );

    createTrackbar(Erosion_element,
                    main_window_name, &erosion_elem,
                     erosion_elem_max, Image_processing_function);

    createTrackbar(Erosion_size,
                    main_window_name, &erosion_size,
                     erosion_size_max, Image_processing_function);

    createTrackbar(Dilation_element,
                    main_window_name, &dilation_elem,
                      dilation_elem_max, Image_processing_function);

    createTrackbar(Dilation_size,
                    main_window_name, &dilation_size,
                      dilation_size_max, Image_processing_function);


    // initializing function
    Image_processing_function( 0, 0 );

    // Waiting for key from user to terminate the program
    while(true)
    {
        int c;
        c = waitKey( 20 );
        if( (char)c == 27 )
            { break; }
    }

}
