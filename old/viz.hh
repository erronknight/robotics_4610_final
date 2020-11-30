#ifndef VIZ_H
#define VIZ_H


#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

void cam_init();
void cam_show(cv::Mat);
int viz_run(int argc, char **argv);
int viz_hit(float range, float angle, float r, float g, float b);

#endif
