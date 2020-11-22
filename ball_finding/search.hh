// Not super sure where you wanted me to put this, but I hope this will prove useful
//
// - Thomas

#ifndef SEARCH_HH
#define SEARCH_HH

// I'm kind of assuming this exists... someone fix this if it's boned lmfao
#include "robot.hh"

// Imports OpenCV utilities
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

// Finds the direction of the ball relative to the current angle given
// the robot's view.
//
// NOTE: If the ball is not found, this returns 666
// (I would do -1, but that's technically a valid angle).
float get_direction_of_ball(cv::Mat pic, float current_angle);

// If the ball is in the camera frame, this will return the distance to get to
// said ball
//
// NOTE: If the ball is not found, this returns -1
float get_distance_from_ball(cv::Mat pic);

// Use this in your finite state machine to begin searching for the ball if
// it's nowhere in sight
//
// Returns true if the ball is in sight
bool search_for_ball(Robot* robot);

// Use this in your finite state machine after search_for_ball to turn towards
// the ball when it is in the camera frame.
//
// Returns true when robot is facing ball within 'tolerance' radians
bool turn_to_ball(Robot* robot, float tolerance);


#endif

