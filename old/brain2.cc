
#include <iostream>
#include <math.h>
#include <string>

#include "robot.hh"
#include "cam.hh"

using std::cout;
using std::endl;

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/

using namespace cv;
using namespace std;

int threshold_value = 55;
int threshold_type = 0;
int const max_value = 255;
int const max_type = 4;
int const max_binary_value = 255;
Mat src, src_gray, dst;
const char* window_name = "Threshold Demo";
const char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
const char* trackbar_value = "Value";
static void Threshold_Demo( int, void* )
{
    /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
    */
    threshold( src_gray, dst, threshold_value, max_binary_value, threshold_type );
    
}

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


void
top_bottom(Mat& img, int& top, int& bottom, int c_start, int c_end) {
    
    int min_val = img.rows + 1;
    int max_val = -1;
    
    for(int i = 0; i < img.rows; i++) {
        uchar* ptr = img.ptr<uchar>(i);

        for(int j = c_start; j < c_end; j++) {
            int val = ptr[j];
            if (val == 0) {
                ptr[j] = 200;
                // cout << Mi[j] << " " << endl;
                max_val = std::max(i, max_val);
                min_val = std::min(i, min_val);
            }
        }
    }
    top = max_val;
    bottom = min_val;

    for(int i = 0; i < img.rows; i++) {
        uchar* ptr = img.ptr<uchar>(i);
        for(int j = c_start; j < c_end; j++) {
            if (i == top) {
                ptr[j] = 128;
            }

            if (i == bottom) {
                ptr[j] = 64;
            }
        }
    }


    // cout << endl;
}

void
callback(Robot* robot) {
    src = robot->frame;
    cam_show(robot->frame);

    robot->set_vel(0.0, 0.0);
    if (!src.empty()) {

        cvtColor( src, src_gray, COLOR_BGR2GRAY ); // Convert the image to Gray
        namedWindow( window_name, WINDOW_AUTOSIZE ); // Create a window to display results
        createTrackbar( trackbar_type,
                        window_name, &threshold_type,
                        max_type, Threshold_Demo ); // Create a Trackbar to choose type of Threshold
        createTrackbar( trackbar_value,
                        window_name, &threshold_value,
                        max_value, Threshold_Demo ); // Create a Trackbar to choose Threshold value
        Threshold_Demo( 0, 0 ); // Call the function to initialize
        // waitKey();
        int top_val = 0;
        int bottom_val = 0;
        
        // string ty =  type2str( dst.type() );
        // printf("Matrix: %s %dx%d \n", ty.c_str(), dst.cols, dst.rows );

        int segments = 100;
        int div = dst.cols/segments;
        cout << div << endl;


        int largest_size = 0;

        for (int i = 0; i < segments; i += 1)
        {
            top_bottom(dst, top_val, bottom_val, i * div, (i + 1) * div);
            float ang =  (((80.0/(segments - 1)) * i) - 40.0);
            // cout << top_val << " " << bottom_val << endl;

            int dist = (top_val - bottom_val);
            largest_size = std::max(largest_size, dist);
        }
        

        cout << robot->range << " " << largest_size << endl;

        imshow( window_name, dst );
    }
}




int
main(int argc, char* argv[]) {
    cam_init();
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
