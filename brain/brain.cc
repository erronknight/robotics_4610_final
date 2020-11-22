
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <deque>
#include <chrono>
#include <cmath>

#include "robot.hh"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <mutex>

/* Surface to store current scribbles */
std::mutex mx2;
 
using std::cout;
using std::endl;


#include <string>

/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/

using namespace cv;
using namespace std;

int threshold_value = 53;
int threshold_type = 0;
int const max_value = 255;
int const max_type = 4;
int const max_binary_value = 255;
Mat src, src_gray, dst;
Mat map_img(800, 800, CV_8UC3, Scalar(255, 255, 255));
const char* window_name = "Threshold Demo";
const char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
const char* trackbar_value = "Value";

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


std::vector<std::vector<bool>> hit_lookup;
std::vector<float> coll = {0, 0, 0};
std::vector<float> drive_shift = {0, 0, 0};
std::vector<float> save_guess = {0, 0, 0};

#define PI 3.14159265


void print_arr(std::vector<float> v) {
  for (auto elem : v) {
    cout << elem << " ";
  }
  cout << endl;
}

std::vector<float> guess = {0, 0, 0};
std::vector<float> offset = {0, 0, 0};

bool init_val = false;
bool coll_delete = false;
float time_delt = 0;
float time_delta = 0.1;


float grid_div = 0.19f;
int grid_offset = 30;
int t_size = std::floor((float(grid_offset) * 2.0)/grid_div);
float radius = float(grid_offset);
float map_scale = 800.0/t_size;

void blot(int x, int y, float r, float g, float b, int count) {
  for (int i = -count; i < count; ++i)
  {
    for (int j = -count; j < count; ++j)
    {
      int nx = i + x;
      int ny = j + y;
      if (nx >= 800 || nx < 0 || ny >= 800 || ny < 0) {
        continue;
      }
      map_img.at<Vec3b>(nx, ny) = Vec3b(floor(b * 255), floor(g * 255), floor(r * 255));
    }
  }
  
}


void draw_point(float x, float y, float r, float g, float b) {

  int x_delt = 800 - floor(x * map_scale);
  int y_delt = 800 - floor(y * map_scale);

  blot(x_delt, y_delt, r, g, b, 1);
}

void draw_path (std::deque<std::vector<int>> d_path) {
  for (int i = 0; i < d_path.size(); ++i)
  {
    std::vector<int> point = d_path[i];
    draw_point(point[0], point[1], 0.0f, 1.0f, 1.0f);
  }
}


std::vector<std::vector<float>> master_wall_points_dirty;
std::vector<std::vector<float>> master_wall_points_clean;
std::vector<std::vector<float>> master_car_points;
std::vector<float> err_input = {0, 0, 0};

bool stop = false;
bool setup = true;

float chassis_dy   = 0.216f;
float wheel_radius = 0.050f;
float wheel_width  = 0.040f;
float wheel_y0     = (chassis_dy*0.5f + wheel_width*0.6f + 0.010f);
float wheel_tread = wheel_y0 * 2;

float l_slp = 0.1f;
float r_slp = 0.1f;
float cg_y = 0;

bool last_cheat_set = false; 
float last_time = 0;

std::vector<float> last_drive = {0, 0};
std::vector<float> guess_diff = {0, 0};
std::vector<float> last_heading = {0, 0};

std::vector<float> last_drive_input= {2, 2};

void cmd_vel(Robot* robot, float vx, float vy, bool print) {


  float tot = guess_diff[0] * guess_diff[0] + guess_diff[1] * guess_diff[1];
  float coll_tot = coll[0] * coll[0] + coll[1] * coll[1];

  float pred_vec = guess_diff[0] * coll[0] + coll[1] * guess_diff[1];
  float diff_scal = ((guess_diff[0] - coll[0]) * (guess_diff[0] - coll[0])) + ((guess_diff[1] - coll[1]) * (guess_diff[1] - coll[1]));
  float d_s = 0.02 * clamp(1, diff_scal / 0.01f, 5);

  if (coll_delete && ((abs(pred_vec) > 0 && coll_tot > 0) || abs(coll[2] - guess_diff[2]) > 0.001)){
    float a1 = atan2(coll[1], coll[0]);
    float a2 = atan2(guess_diff[1], guess_diff[0]);
    float tang = (a1 - a2);

    if (guess_diff[2] > PI) {
      guess_diff[2] -= PI * 2;
    } else if (guess_diff[2] < -PI) {
      guess_diff[2] += PI * 2;
    }

    if (tang > PI) {
      tang -= PI * 2;
    } else if (tang < -PI) {
      tang += PI * 2;
    }

    if ((tang > 0.1 && abs(last_drive[0] + last_drive[1]) / 2.0f > 0.1f) || coll[2] < guess_diff[2]) {
      drive_shift[1] += d_s;
      drive_shift[0] -= d_s;
    }

    if ((tang < -0.1 && abs(last_drive[0] + last_drive[1]) / 2.0f > 0.1f ) || coll[2] > guess_diff[2]) {
      drive_shift[1] -= d_s;
      drive_shift[0] += d_s;
    }

    cout << guess_diff[0] << " " << guess_diff[1] << endl;
    cout << coll[0] << " " << coll[1] << endl;
    
    last_drive = {0, 0};


    if (tot < coll_tot - 0.001 || pred_vec < 0) {
        drive_shift[0] -= d_s;
        drive_shift[1] -= d_s;
    } else if (tot > coll_tot + 0.001) {
        drive_shift[0] += d_s;
        drive_shift[1] += d_s;
    }

  }

  last_drive = {last_drive[0] + vx, last_drive[1] + vy};

  if (coll_delete) {
    coll_delete = false;
    coll[0] = 0;
    coll[1] = 0;
    coll[2] = 0; 
  }

  // if (abs(time_delta) < 1000) {
    coll[0] +=  ((vx + vy) / 2.0f * time_delta * wheel_radius * PI * 2 * 0.68 * cos(guess[2]));
    coll[1] +=  ((vx + vy) / 2.0f * time_delta * wheel_radius * PI * 2 * 0.68 * sin(guess[2]));
    coll[2] += -((vx - vy) / 2.0f * time_delta * wheel_radius * PI * 2 * 0.68 / (wheel_y0 * PI * 2));
  // }

  drive_shift[0] = clamp((-10.0f * 0.2f) / 1.05f,  drive_shift[0], (10.0f * 0.2f) / 1.05f);
  drive_shift[1] = clamp((-10.0f * 0.2f) / 1.05f,  drive_shift[1], (10.0f * 0.2f) / 1.05f);
  robot->set_vel(vx, vy);

  // robot->set_vel(vx / 1.05f - drive_shift[0], vy / 1.05f - drive_shift[1]);
}

int pre_test = 4;
int wait_test = 20;
int test_length = 30;
int test_count = 0;
bool not_validated = true;
int col_frq = 2;
bool passed_validation = false;
std::vector<std::vector<float>> tile_grid(t_size, std::vector<float>(t_size, -1));
std::vector<std::vector<float>> tile_grid_thread(t_size, std::vector<float>(t_size, -1));

void apply_adj(int x, int y, float v, std::vector<std::vector<float>> &grid) {
  int ax = x + 1;
  int ay = y + 0;

  int bx = x - 1;
  int by = y + 0;

  int cx = x + 0;
  int cy = y + 1;

  int dx = x + 0;
  int dy = y - 1;

  if (ax >= 0 && ay >= 0 && ax < grid.size() && ay < grid[0].size()) {
    if (grid[ax][ay] == -1) {
      grid[ax][ay] = 0;
    }
    grid[ax][ay] += v;
    grid[ax][ay] = clamp(-0.99, grid[ax][ay], 2.0);
  }

  if (bx >= 0 && by >= 0 && bx < grid.size() && by < grid[0].size()) {
    if (grid[bx][by] == -1) {
      grid[bx][by] = 0;
    }
    grid[bx][by] += v;
    grid[bx][by] = clamp(-0.99, grid[bx][by], 2.0);
  }

  if (cx >= 0 && cy >= 0 && cx < grid.size() && cy < grid[0].size()) {
    if (grid[cx][cy] == -1) {
      grid[cx][cy] = 0;
    }
    grid[cx][cy] += v;
    grid[cx][cy] = clamp(-0.99, grid[cx][cy], 2.0);
  }

  if (dx >= 0 && dy >= 0 && dx < grid.size() && dy < grid[0].size()) {
    if (grid[dx][dy] == -1) {
      grid[dx][dy] = 0;
    }
    grid[dx][dy] += v;
    grid[dx][dy] = clamp(-0.99, grid[dx][dy], 2.0);
  }
}

bool is_adj(int x, int y, int v, std::vector<std::vector<float>> &grid, int d) {

  int width = grid.size();
  int height = grid[0].size();

  for (int xs = -d; xs <= d; ++xs)
  {
    for (int ys = -d; ys <= d; ++ys)
    {
      int tx = x + xs;
      int ty = y + ys;
      if (tx >= 0 && ty >= 0 && tx < width && ty < height) {
        if (tile_grid[tx][ty] > v) {
          return true;
        }
      }
    }
  }
  return false;
}

bool plot_point_b(int x, int y, bool add, std::vector<std::vector<float>> &grid, float draw) {

  if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size()) {
    return true;
  }

  if (grid[x][y] == -1) {
    grid[x][y] = 0;
  }

  if (grid[x][y] < -0.9) {
    grid[x][y] = -0.9;
  }

  if (add) {
    grid[x][y] += 0.45f;
    apply_adj(x, y, 0.1, grid);
  } else {
    grid[x][y] -= 0.45f;
    apply_adj(x, y, -0.1, grid);
    //apply_adj(x, y, -0.15, grid);
  }

  grid[x][y] = clamp(-0.99, grid[x][y], 2.0);

  if (draw) {
    draw_point(x, y, clamp(0, grid[x][y], 1.0f) / 1.0f, 0, 0);
  }
  return false;
}

bool check_val(int x, int y, float val, bool adj, std::vector<std::vector<float>> &grid, int range) {
  if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size()) {
    return true;
  }

  if (grid[x][y] > val || (adj && is_adj(x, y, val, grid, range))){
    return true;
  }

  return false;
}


bool bresenham_int(int x1, int y1, int x2, int y2, bool hit, bool find, std::vector<std::vector<float>> &grid, bool draw, int range) { 
  int delta_x(x2 - x1);
  // if x1 == x2, then it does not matter what we set here
  signed char const ix((delta_x > 0) - (delta_x < 0));
  delta_x = std::abs(delta_x) << 1;

  int delta_y(y2 - y1);
  // if y1 == y2, then it does not matter what we set here
  signed char const iy((delta_y > 0) - (delta_y < 0));
  delta_y = std::abs(delta_y) << 1;
  
  if (!find && plot_point_b(x1, y1, false, grid, draw)) {
    return false;
  }

  if (find && check_val(x1, y1, 0, false, grid, 0)) {
    cout << "o1:" << x1 << " " << y1  << " " << tile_grid[x1][y1] <<  endl;
    return false;
  }

  bool init = false;
  int last_x = 0;
  int last_y = 0;

  if (delta_x >= delta_y)
  {
      // error may go below zero
      int error(delta_y - (delta_x >> 1));

      while (x1 != x2)
      {
          if (find && init && check_val(last_x, last_y, 0, true, grid, range)) {
            cout << "l1:" << last_x << " " << last_y <<  endl;
            return false;
          }

          if (!find && init && plot_point_b(last_x, last_y, false, grid, draw)) {
            return false;
          }

          // reduce error, while taking into account the corner case of error == 0
          if ((error > 0) || (!error && (ix > 0)))
          {
              error -= delta_x;
              y1 += iy;
          }
          // else do nothing

          error += delta_y;
          x1 += ix;

          last_x = x1;
          last_y = y1;
          
          init = true;
      }
  }
  else
  {
      // error may go below zero
      int error(delta_x - (delta_y >> 1));

      while (y1 != y2)
      {
          if (find && init && check_val(last_x, last_y, 0, true,  grid, range)) {
            cout << "l2:" << last_x << " " << last_y <<  endl;
            return false;
          }

          if (!find && init && plot_point_b(last_x, last_y, false, grid, draw)) {
            return false;
          }
          // reduce error, while taking into account the corner case of error == 0
          if ((error > 0) || (!error && (iy > 0)))
          {
              error -= delta_y;
              x1 += ix;
          }
          // else do nothing

          error += delta_x;
          y1 += iy;

          last_x = x1;
          last_y = y1;
          
          init = true;
      }
  } 
  
  if (last_x < 0 || last_y < 0 || last_x >= t_size || last_y >= t_size) {

    cout << "l3:" << last_x << " " << last_y <<  endl;
    return false;
  }



  if (!find) {
    if (hit) {
      plot_point_b(last_x, last_y, true, grid, draw);
    } else if (tile_grid[last_x][last_y] < 0) {
      plot_point_b(last_x, last_y, false, grid, draw);
    }
  }

  if (find && init && check_val(last_x, last_y, 0, false,  grid, range)) {
    cout << "l4:" << last_x << " " << last_y <<  endl;
    return false;
  }

  return true;
}

// code modified from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
bool bresenham(float xf1, float yf1, float xf2, float yf2, bool hit, bool find, std::vector<std::vector<float>> &grid, bool draw, int range) { 
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  if (!find) {
    x1 = std::floor((grid_offset + xf1) / grid_div);
    y1 = std::floor((grid_offset + yf1) / grid_div);
    x2 = std::floor((grid_offset + xf2) / grid_div);
    y2 = std::floor((grid_offset + yf2) / grid_div);
  } else {
    x1 = xf1;
    y1 = yf1;
    x2 = xf2;
    y2 = yf2;
  }
  
  return bresenham_int(x1, y1, x2, y2, hit, find, grid, draw, range);
} 


std::deque<std::vector<int>> reduce_path(std::deque<std::vector<int>> input) {
  cout << " reduce " << input.size() << endl;

  for (int i = 0; i < input.size() - 1; ++i) {
    std::vector<int> start = input[i];

    if (i >= input.size() - 1) {
      break;
    }

    for (int j = i + 1; j < input.size() - 1; ++j) {
      std::vector<int> end = input[j];
      // cout << start[0] << " " << start[1] << " " <<  end[0] <<  " " <<  end[1] << endl;
      if (bresenham(start[0], start[1], end[0], end[1], false, true, tile_grid, false, 5)) {
        // cout << "del:"  << j << endl;
        input.erase(input.begin() + j);
        j--;
      } else {
        i = j - 1;
        break;
      }
    }
  }

  for (int i = 0; i < input.size(); ++i)
  {
    std::vector<int> start = input[i];
    float dx = float(start[0] * grid_div) - float(grid_offset);
    float dy = float(start[1] * grid_div) - float(grid_offset);
    cout << dx <<  " " << dy << endl;
  }

  cout << " end " << input.size() << endl;
  return input;
}

bool near_goal(int x, int y, int goal_x, int goal_y) {
  if ((x - goal_x) * (x - goal_x) + (y - goal_y) * (y - goal_y) < 5) {
    return true;
  }
  return false;
}


bool add_path (std::vector<std::deque<std::vector<int>>>* paths, int path_idx, 
  bool* used, std::deque<std::vector<int>>* found_path,
  std::vector<std::vector<bool>>* not_checked, std::vector<std::vector<float>> &grid,
  int x, int y, int x2, int y2, bool not_finished) {

  // cout << x << " + " << y << " " << x2 << " + " << y2 << endl;

  if (x < 0 || y < 0 || x >= t_size || y >= t_size) {
    return false;
  }

  if ((*not_checked)[x][y]) {
      (*not_checked)[x][y] = false;


      if (is_adj(x, y, 0, grid, 2) && !near_goal(x, y, x2, y2)) {
        return false;
      }

      std::vector<int> point = {x, y};

      if (*used) {
        std::deque<std::vector<int>> new_path((*paths)[path_idx]);
        new_path.emplace_front(point);
        paths->emplace_back(new_path);

        if (near_goal(x, y, x2, y2) && grid[x][y] < 0) {
          *found_path = new_path;
          return true;
        }
      } else {
        
        if (not_finished) {
          std::deque<std::vector<int>> new_path((*paths)[path_idx]);
          paths->emplace_back(new_path);
        }

        (*paths)[path_idx].emplace_front(point);
        *used = true;

        if (near_goal(x, y, x2, y2) && grid[x][y] < 0) {
          *found_path = (*paths)[path_idx];
          return true;
        }
      }
    }

    return false;
}

std::vector<float> last_pos = {0, 0};
std::vector<float> this_pos = {0, 0};

std::vector<float> last_guess = {0, 0};
std::vector<int> last_err = {0, 0};
int c = 0;


void fix_guess(Robot* robot) {
  // cout << robot->err_x << " " << robot->err_y << endl;
  // if (abs(time_delta) < 1000) {
  //   return;
  // }
  cout << "A" << endl;


  int x1 = std::floor((grid_offset + guess[0]) / grid_div);
  int y1 = std::floor((grid_offset + guess[1]) / grid_div);
  int range = 5;//std::ceil(4.0 / grid_div);

  float w_range = 2.5;
  int window = std::ceil((w_range * 2 + grid_div) / grid_div);
  int window_shift = std::ceil(window / 2.0f);

  std::vector<std::vector<float>> relative_map(window, std::vector<float>(window, -1.0f));;

 
  float shift_x = 0;
  float shift_y = 0;

  int best_x = 0;
  int best_y = 0;

  float min_err = FLT_MAX;
  float max_gain = -FLT_MAX;
  float min_err_none = 0.15;

  std::vector<float> c_col = coll;
  // if (abs(time_delta) < 1000) {
    c_col = {0, 0, 0};
  // } else {
    last_heading = {guess[2]};
    coll_delete = true;
  // }

  

  for (int i1 = -range; i1 <= range; ++i1)
  {
    for (int i2 = -range; i2 <= range; ++i2)
    {
      float err = 0;
      float hit = 0;
      for (int j1 = 0; j1 < window; ++j1)
      {
        for (int j2 = 0; j2 < window; ++j2)
        {
          int x_actual = x1 - window_shift + i1 + j1;
          int y_actual = y1 - window_shift + i2 + j2;

          if (x_actual < 0 || x_actual >= t_size || y_actual < 0 || y_actual >= t_size) {
            continue;
          }

          if (relative_map[j1][j2] == -1 || tile_grid[x_actual][y_actual] == -1) {
            continue;
          } else if ((relative_map[j1][j2] > 0) ^ (tile_grid[x_actual][y_actual] > 0)) {
            err += abs(relative_map[j1][j2] * tile_grid[x_actual][y_actual]);
          } else {
            hit += abs(relative_map[j1][j2] * tile_grid[x_actual][y_actual]);
          }
        }
      }

      float cum_err = abs((guess[0] + i1 * 0.02) - (last_guess[0] + c_col[0])) * 20.0f
          + abs((guess[1] + i2 * 0.02) - (last_guess[1] + c_col[1])) * 20.0f + err - (hit/10.0f);

      // cout << "i1:" << i1 << "\ti2:" << i2 << "\tdx:" << abs((guess[0] + i1 * grid_div) - (last_guess[0] + coll[0])) * 5 << "\tdy:" 
      //     << abs((guess[1] + i2 * grid_div) - (last_guess[1] + coll[1])) * 5 << "\terr:" << err << "\thit:" << (hit/10.0f) << "\tcum:" << cum_err << endl;

      if (hit > 0) {
        if (cum_err < min_err) {
          best_x = i1;
          best_y = i2;
          min_err = cum_err;
          max_gain = hit;
        }
      }
    }
  }

  if (abs(guess[0] - last_guess [0]) < 0.001) {
    best_x = 0;
  } 
  if (abs(guess[1] - last_guess [1]) < 0.001) {
    best_y = 0;
  }

  offset[0] += best_x * 0.02;
  offset[1] += best_y * 0.02;

  offset[0] = clamp(-0.2f,  offset[0], 0.2f);
  offset[1] = clamp(-0.2f,  offset[1], 0.2f);


  // last_err = {robot->err_x, robot->err_y};

  last_guess = {robot->pos_x + offset[0], robot->pos_y + offset[1], robot->pos_t};

  // if (abs(time_delta) > 1000) {
    guess_diff = {last_guess[0] - save_guess[0], last_guess[1] - save_guess[1], last_guess[2] - save_guess[2]};
    save_guess = {last_guess[0], last_guess[1], last_guess[2]};
  // }

  // drive_shift = {guess[0] - (last_guess[0] + c_col[0]), guess[1] - (last_guess[1] + c_col[1])};
  
  // c++;
  // if (c > 100) {
  //   exit(0);
  // } 
}

int check_count = 0;
int check_limit = 25;
int check_count2 = 0;
int check_limit2 = 50;

std::deque<std::vector<int>> target_path;
std::deque<std::vector<int>> target_path_thread;
std::vector<int> target_point = {};
bool has_path = false;

bool hit_drive = false;

bool drive_to_point (Robot* robot, std::vector<int> point) {
  draw_point(point[0], point[1], 0.0f, 1.0f, 0.0f);
  float dx = (float(point[0] * grid_div) - grid_offset) - guess[0];
  float dy = (float(point[1] * grid_div) - grid_offset) - guess[1];
  float ang = atan2(dy, dx);
  float dist = dx * dx + dy * dy;
  float tang = ang - guess[2];
  if (tang > PI) {
    tang -= PI * 2;
  } else if (tang < -PI) {
    tang += PI * 2;
  }

  float c_dist = 0.35;
  float fwd = robot->range;
  bool flag = fwd < 0.3;


  float speed = 2.5f;
  float turn = 0.5f;

  if (dist < 2 * 2) {
    return true;
  } else if (tang < -0.20){
    if (dist > 4 * 4 && tang > -0.35 && hit_drive) {
      cmd_vel(robot, speed + turn, speed - turn, false);
    } else {
      cmd_vel(robot, speed, -speed, false);
    }
     if (flag) {
      return true;
    } 
    cout << "rotating " << ang << " " << guess[2]  << " " << tang << endl;
    return false;
  } else if (tang > 0.20){
    if (dist > 4 * 4 && tang < 0.35 && hit_drive) {
      cmd_vel(robot, speed - turn, speed + turn, false);
    } else {
      cmd_vel(robot, -speed, speed, false);
    }
    cout << "rotating " << ang << " " << guess[2] << " " << tang << endl;
     if (flag) {
      return true;
    } 
    return false;
  } else if (dist > 2 * 2) {
    cout << "driving" << ang << " " << guess[2] << " " << tang << endl;
    hit_drive = true;
    if (flag) {
      return true;
    } 
    cmd_vel(robot, speed, speed, false);
    return false;
  } 

  hit_drive = false;

  return true;
}

bool left_follow = false;
bool right_follow = false;
bool searching = false;

std::vector<float> thread_pos = {0, 0};

void main_drive(Robot* robot) {
  //cout << "validating" << endl;
  //bool result = validate_coords(dirty_input);
  //cout << "validated" << endl;
  guess = {robot->pos_x + offset[0], robot->pos_y + offset[1], robot->pos_t};
  // cout << coll[0] << " = "<< coll[1] <<  " " << time_delta << endl;
  // cout << save_guess[0] - guess[0] << " - "<< save_guess[1] - guess[1] <<  " " << time_delta << endl;
  // fix_guess(robot);
  cout << "go" << endl;
  guess = {robot->pos_x + offset[0], robot->pos_y + offset[1], robot->pos_t};

  // if (check_count > check_limit && !result) {
  //   cmd_vel(robot, 0.0, 0.0, false);
  // } else {

  // if (check_count > 20) {
  //   cmd_vel(robot, 0, 0, false);
  //   check_count = 0;
  //   return;
  // }

  src = robot->frame;
  cout << " 2 " << t_size << " {} " << endl; 

  if (!src.empty()) {

    

      // cvtColor( src, src_gray, COLOR_BGR2GRAY ); // Convert the image to Gray
    cvtColor( src, src_gray, COLOR_BGR2GRAY ); // Convert the image to Gray
    // namedWindow( window_name, WINDOW_AUTOSIZE );
    threshold( src_gray, dst, threshold_value, max_binary_value, threshold_type );
    // Threshold_Demo( 0, 0 ); // Call the function to initialize
    // waitKey();
    int top_val = 0;
    int bottom_val = 0;

    // string ty =  type2str( dst.type() );
    // printf("Matrix: %s %dx%d \n", ty.c_str(), dst.cols, dst.rows );

    int segments = 40;
    int div = dst.cols/segments;

    for (int i = 0; i < segments; i += 1)
    {
      top_bottom(dst, top_val, bottom_val, i * div, (i + 1) * div);
      int dist = (top_val - bottom_val);

      float actual_dist = 461.6 * std::pow(dist, -0.9965);
      float ang = -(((80.0/(segments - 1)) * i) - 40.0) * 0.0174533;
      float x_val1 = guess[0] + actual_dist * cos(ang + guess[2]);
      float y_val1 = guess[1] + actual_dist * sin(ang + guess[2]);
          

      if (dist > 50 && dist < 530) {
          bresenham(guess[0], guess[1], x_val1, y_val1, true, false, tile_grid, true, 1);
      }

      // if (dist <= 50) {
      //     bresenham(guess[0], guess[1], x_val1, y_val1, false, false, tile_grid, true, 1);
      // }
      
    }
    Mat img_print(800, 800, CV_8UC3, Scalar(0,0, 100));
   
    img_print.at<Vec3b>(0, 0) = Vec3b(0, 255, 0);
    img_print.at<Vec3b>(799, 799) = Vec3b(0, 255, 0);


    cv::imshow("camera", dst);
    cv::imshow("map", map_img);
    cv::waitKey(1);
  }
  cout << " 3 " << dst.size().width << endl; 


  float c_dist = 0.35;

  float fwd = robot->range;

  bool flag = fwd < 0.3;

  // for (int i = 0; i < target_path.size(); ++i)
  // {
  //   float dx = float(target_path[i][0] * grid_div) - float(grid_offset);
  //   float dy = float(target_path[i][1] * grid_div) - float(grid_offset);
  //   cout << i << " " << target_path[i][0] << " " << target_path[i][1] << " " << dx << " " << dy << endl; 
  // }

  // if (check_count2 > check_limit2 ) {
    if (has_path) {
      float dx = float(target_point[0] * grid_div) - float(grid_offset);
      float dy = float(target_point[1] * grid_div) - float(grid_offset);
      int sx = std::floor((grid_offset + guess[0]) / grid_div);
      int sy = std::floor((grid_offset + guess[1]) / grid_div);

      cout << "driving to = [" << dx << ", " << dy << "] p_left: [" << target_path.size() << "] current = [" << guess[0] << " " <<  guess[1] << "] " << thread_pos[0] << " " << thread_pos[1] << endl;
      bool flag2 = flag || !bresenham_int(sx, sy, target_point[0], target_point[1], false, true, tile_grid, false, 0);
      if (tile_grid[target_point[0]][target_point[1]] > 0.5 || flag2) {
        cout << "path_updated:" << tile_grid[target_point[0]][target_point[1]] << " " << flag2 << " " << flag << endl;
        check_count2 = 0;
        has_path = false;
      } else if (drive_to_point(robot, target_point)) {
        if (target_path.size() > 0) {
          target_point = target_path.back();
          target_path.pop_back();
        } else {
          target_point.clear();
          target_path.clear();
          check_count2 = 0;
          has_path = false;
        }
      }

      cout << "?" << endl;
      return;
    } else {
      cout << "path_cleared" << endl;
      has_path = false;

      target_point.clear();
      check_count2 = 0;
    }
    check_count2 = check_limit2;
  // }

  cout << "cc:" << check_count2  << " p?:" << has_path << " s?:" << searching << endl;
  check_count2++;

  
  
  float speed = 2.5f;
  float turn = 1.5f;
  float f_min = 0.6f;
  float f2_min = 0.75f;
  float f3_min = 0.9f;

  if (fwd) {
    cmd_vel(robot, -speed, speed, false);
  } else {
    cmd_vel(robot, speed, speed, false);
  }
}
int setup_c = 0;
bool at_goal = false;
bool at_goal2 = false;
bool new_goal = false;

void callback(Robot* robot) {
  if (setup_c < 20) {
    setup_c++;
    return;
  }

  at_goal = robot->at_goal();

  if (mx2.try_lock()) {

    if (new_goal) {
      target_path.clear();


      for (int i = 0; i < target_path_thread.size(); ++i)
      {
        std::vector<int> p = {target_path_thread[i][0], target_path_thread[i][1]};
        target_path.emplace_back(p);
      }

      if (target_path.size() > 0) {
        target_point = target_path.back();
        target_path.pop_back();

        has_path = true;
      }

      new_goal = false;
    }

    for (int i = 0; i < t_size; ++i)
    {
      for (int i2 = 0; i2 < t_size; ++i2)
      {
        tile_grid_thread[i][i2] = tile_grid[i][i2];
      }
    }

    thread_pos = {guess[0], guess[1]};

    mx2.unlock();
  }

  main_drive(robot);
  // } 
}

std::deque<std::vector<int>> get_target(float xf, float yf, float t_x, float t_y, std::vector<std::vector<float>> &grid) {
  std::vector<std::vector<bool>> not_checked(t_size, std::vector<bool>(t_size, true));
  std::vector<std::deque<std::vector<int>>> paths;

  int x1 = std::floor((grid_offset + xf) / grid_div);
  int y1 = std::floor((grid_offset + yf) / grid_div);
  int x2 = std::floor((grid_offset + t_x) / grid_div);
  int y2 = std::floor((grid_offset + t_y) / grid_div);
  std::deque<std::vector<int>> path_try = {{x1, y1}};

  paths.emplace_back(path_try);
  int path_idx = 0;
  std::deque<std::vector<int>> found_path = {{x1, y1}};
  while (path_idx < paths.size()) {
    bool used = false;

    int xo = paths[path_idx][0][0];
    int yo = paths[path_idx][0][1];

    int ax = xo + 0;
    int ay = yo + 1;

    int bx = xo + 1;
    int by = yo + 0;

    int cx = xo + 0;
    int cy = yo - 1;

    int dx = xo - 1;
    int dy = yo + 0;


    std::vector<std::vector<int>> n2 = {{ax, ay}, {bx, by}, {cx, cy}, {dx, dy}};
    std::vector<int> order = {0, 1, 2, 3};
    std::vector<float> orderf = {0, 1, 2, 3};
    std::vector<float> dists = {0, 0, 0, 0};

    for (int i = 0; i < 3; ++i)
    {
      for (int i = 0; i < n2.size() - 1; ++i)
      {
        std::vector<int> p1 = n2[order[i]];
        std::vector<int> p2 = n2[order[i + 1]];

        float v1 = ((p1[0] - x2) * (p1[0] - x2)) + ((p1[1] - y2) * (p1[1] - y2));
        float v2 = ((p2[0] - x2) * (p2[0] - x2)) + ((p2[1] - y2) * (p2[1] - y2));
        dists[order[i]] = v1;
        dists[order[i + 1]] = v2;

        if (v1 > v2) {
          int tmp = order[i];
          order[i] = order[i + 1];
          order[i + 1] = tmp;
        }
      }
    }

    for (int i = 0; i < 4; ++i)
    {
      orderf[i] = float(order[i]);
    }

    // print_arr(orderf);
    // print_arr(dists);

    bool end = false;

    for (int i = 0; i < 4; ++i)
    {
      int idx = order[i];
      if (add_path(&paths, path_idx, &used, &found_path, &not_checked, grid, n2[idx][0], n2[idx][1], x2, y2, i < 3)){
        end = true;
        break;
      }
    }
    

    if (end) {
      break;
    } 

    if (!used) {
      path_idx += 1;
    }

  }

  draw_path(found_path);
  return found_path;
}


void calc_path() {
  cout << "Thread spawned" << endl;
  while (!at_goal) {
    if (!new_goal && !has_path) {
      mx2.lock();
      at_goal2 = at_goal;
      searching = true;

      target_path_thread = reduce_path(get_target(thread_pos[0], thread_pos[1], 20, 0, tile_grid_thread));
      new_goal = true;
      searching = false;
      mx2.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

#include <X11/Xlib.h>


int
main(int argc, char* argv[])
{
    XInitThreads();
    
    cout << "making robot" << endl;

    std::thread first(calc_path);

    

    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
