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

////////////////////////////////////////////
// CONSTANTS (please validate with model) //
////////////////////////////////////////////

#define fov                   1.3962634 // Field of View    TODO

#define ball_hue              32/2      // Orange ball!
#define color_tolerance       8/2       // +/- color tolerance

#define ball_d                1         // Diameter of the ball (in meters) TODO

#define turn_speed            1.5       // Wheel speed for self-rotation
#define ball_pixel_threshold  5
#define image_search_edge     10
#define DEBUG                 false

////////////////////////////////////////////
//                                        //
////////////////////////////////////////////

// Finds the direction of the ball relative to the current angle given
// the robot's view.
//
// NOTE: If the ball is not found, this returns 666
// (I would do -1, but that's technically a valid angle).
float get_direction_of_ball(cv::Mat pic);
float get_direction_of_ball(Robot* robot);

// If the ball is in the camera frame, this will return the distance to get to
// said ball
//
// NOTE: If the ball is not found, this returns -1
float get_distance_from_ball(cv::Mat pic);
float get_distance_from_ball(Robot* robot);

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

