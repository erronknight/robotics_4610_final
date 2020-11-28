// (hopefully) vaguely useful implementation of search.hh interface
// NOTE: ONCE THE MODEL IS DONE, PLEASE CHECK TODOs

/////////////
// IMPORTS //
/////////////
#include "search.hh"
#include "robot.hh"
#include <iostream>

////////////
// MACROS //
////////////

// Clamps between 2 values
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

//////////////////////
// ADDITIONAL STUBS //
//////////////////////

// Movement helpers
int _facing(Robot* robot, float target_angle, float tolerance);
bool _approx_equal(float a, float b, float tolerance);
float _turn_vel(Robot* robot, float total_turn_angle, float target_angle);
float _diff(float deg1, float deg2);

// CV helpers
cv::Mat _make_ball_mask(cv::Mat pic);
int _get_column_of_ball(cv::Mat pic);
int _ball_pixel_height(int column_index, cv::Mat pic);
float _get_visual_angle(int pixel_height, cv::Mat pic);



//////////////////////////////
// INTERFACE IMPLEMENTATION //
//////////////////////////////

// Finds the direction of the ball relative to the current angle
float get_direction_of_ball(cv::Mat pic) {
    if (pic.empty()){
        std::cout << "IMAGE IS NULL" << std::endl;
        return 666.0;
    }
  
    // Gets a mask of the image filtering the ball
    cv::Mat ball_mask = _make_ball_mask(pic);
    
    
    
    
    // Finds the location of the topmost pixel of the ball (which should also be the center)
    int column = _get_column_of_ball(ball_mask);
    
    if (DEBUG) {
        cv::imshow("raw", pic);
        cv::imshow("filter", ball_mask);
        cv::waitKey(1);
    }
    
    // Returns a sentinel if the ball is not in frame
    if (column < 0) {
        return 666.0;
    }

    // Finds the angle of this column
    int pic_width = pic.cols;
    float proportion = static_cast<float>(column - (pic_width / 2.0)) / static_cast<float>(pic_width / 2.0);
    float direction = proportion * (fov / 2.0);
    return direction;
}
float get_direction_of_ball(Robot* robot) {
    return get_direction_of_ball(robot->frame);
}

// Finds the distance a ball is away given the current picture
float get_distance_from_ball(cv::Mat pic) {
    
    // Gets a mask of the image filtering the ball
    cv::Mat ball_mask = _make_ball_mask(pic);
    
    // Finds the location of the topmost pixel of the ball (which should also be the center)
    int column = _get_column_of_ball(ball_mask);

    // Returns a sentinel if the ball is not in frame
    if (column < 0) {
        return -1.0;
    }

    // Check column for height of pixels of ball
    int ball_pix = _ball_pixel_height(column, ball_mask);

    // With height of ball in pixels, find visual angle
    float visual_angle = _get_visual_angle(ball_pix, pic);
    
    // With visual angle of ball, extrapolate distance with size of ball
    float distance = (static_cast<float>(ball_d) / 2.0) / tan(visual_angle / 2.0);

    // We did it reddit
    return distance;
}

float get_distance_from_ball(Robot* robot) {
    return get_distance_from_ball(robot->frame);
}

// Turns the robot and returns true if the ball is in the camera frame
// TODO: PLEASE CHECK THE ROBOT MODEL
bool search_for_ball(Robot* robot) {
    cv::Mat pic = robot->frame;
    float angle = get_direction_of_ball(pic);
    
    // If angle is 666 instead of a valid angle, start turning
    if (angle > (M_PI / 2.0)) {
        robot->set_vel(-turn_speed, turn_speed);
        return false;
    }

    // The ball has been located!
    robot->set_vel(0.0f, 0.0f);
    return true;
}


// Aligns to the ball when it's in the camera frame
// TODO: PLEASE CHECK THE ROBOT MODEL
bool turn_to_ball(Robot* robot, float tolerance) {
    cv::Mat pic = robot->frame;
    
    float angle = get_direction_of_ball(pic);

    // You are facing the ball!
    if (_approx_equal(angle, 0, tolerance)) {
        robot->set_vel(0.0f, 0.0f);
        return true;
    }

    // Sort of smooths it so you don't massively overshoot
    float spd = clamp(1.0f, 1.0f + 3.0f * angle, 2.0f);

    // Further turning towards the ball!
    (angle < 0) ? robot->set_vel(-spd, spd) : robot->set_vel(spd, -spd);
    return false;
}


////////////////////////////
// HELPERS IMPLEMENTATION //
////////////////////////////

// Returns true if two floats are approximately equal within a tolerance
bool _approx_equal(float a, float b, float tolerance) {
    return abs(a - b) < tolerance;
}


// Returns the difference between two angles, accounting for complete circles
float _diff(float ang_1, float ang_2) {
    float d = ang_1 - ang_2;
    d = d >  (M_PI / 2) ? d - M_PI : d;
    d = d < -(M_PI / 2) ? d + M_PI : d;
    return d;
}


// Makes a mask of the picture that picks out the ball specifically
// Uses HSV to pick out the orange
cv::Mat _make_ball_mask(cv::Mat pic) {
    cv::Mat frame = pic;
    cv::Mat frame_hsv;
    
    if (pic.empty()) {
      return pic;
    }
    
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
    
    cv::Scalar low = cv::Scalar(clamp(0, ball_hue - color_tolerance, 180), 128, 50);
    cv::Scalar high = cv::Scalar(clamp(0, ball_hue + color_tolerance, 180), 255, 255);

    // Creates a black and white mask of the image filtered by the ball color in
    // accordance to the above scalar tolerances
    cv::Mat mask;
    cv::inRange(frame_hsv, low, high, mask);
    
    return mask;
    
}

// Returns the column within the picture that indersects the middle of the ball.
// Returns -1 if there is no mask in frame (i.e. all black)
int _get_column_of_ball(cv::Mat pic) {
    float col_tot = 0;
    float row_tot = 0;
    float count = 0;
    
    // Becasue the object is a circle, the highest up row that has a pixel will subsequently
    // contain the pixel that will be roughly in the direct center of the ball, and can
    // be assumed as the middle
    for (int row = image_search_edge; row < pic.rows - image_search_edge; row++) {
        uchar* ptr2 = pic.ptr<uchar>(row);
        for (int column = image_search_edge; column < pic.cols - image_search_edge; column++) {
            // Found the first highest-up pixel of the mask
            if (ptr2[column]) {
                col_tot += column;
                row_tot += row;
                count += 1;
            }
        }
    }
    
    int col_out = std::floor(col_tot / count);
    int row_out = std::floor(row_tot / count);
    
    if (DEBUG) {
        for (int i = 0; i < pic.rows; i++) {
            uchar *ptr = pic.ptr<uchar>(i);
            ptr[col_out] = 128;
        }
        
        uchar *ptr = pic.ptr<uchar>(row_out);
        for (int j = 0; j < pic.cols; j++) {
            ptr[j] = 64;
        }
    }
    
    if (count > ball_pixel_threshold) {
        return col_out;
    }
    
    return -1;
}


// Given the index of the column where the ball is in the pic, 
int _ball_pixel_height(int column_index, cv::Mat pic) {


    int ball_pix = 0;
    for (int row = 0; row < pic.rows; row++) {
        cv::Vec3b color = pic.at<cv::Vec3b>(cv::Point(column_index, row));

        // // Can find the ball pixel height with just color but this might not be practical
        // if (!using_mask) {
    
        //     // NOTE: COLOR IS IN BGR
        //     if (clamp(0, abs(color.val[0] - ball_b), 255) < color_tolerance
        //         && clamp(0, abs(color.val[0] - ball_g), 255) < color_tolerance
        //         && clamp(0, abs(color.val[0] - ball_r), 255) < color_tolerance) {
        //             ball_pix++;
        //     }
        // }
        // I'm like pretty sure this is how the masked version would work?
        // else {
        if (color.val[0] && color.val[1] && color.val[2]) {
                ball_pix++;
        }
        // }
    }
    return ball_pix;
}


// Given the height of an object in pixels, return the visual angle
float _get_visual_angle(int pixel_height, cv::Mat pic) {
    float proportion = static_cast<float>(pixel_height) / static_cast<float>(pic.rows);
    float visual_angle = proportion * fov;
    return visual_angle;
}





