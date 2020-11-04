#ifndef _NAV_HXX_
#define _NAV_HXX_

#include <vector>

struct Vec2i;
struct Vec2f;

struct Vec2i {
    int x, y;

public:
    Vec2i();
    Vec2i(int x, int y);

    Vec2f to_vec2f() const;

    bool operator==(const Vec2i& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
};

struct Vec2f {
    int x, y;

public:
    Vec2f();
    Vec2f(float x, float y);
    
    Vec2i to_vec2i() const;

    bool operator==(const Vec2f& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
};

#define MAP_WIDTH 60
#define MAP_HEIGHT 60

#define STEP_PROX_THRESH 1
#define BASE_OCCUPIED_THRESH 1
#define CELL_HIT_DIFF 0.3
#define DECAY_STEP_FACTOR 0.9

struct Grid {
    int width;
    int height;
    int cx_off;
    int cy_off;

    // Stored in x-major order.
    std::vector<float> data;

    Grid(int w, int h, int cxo, int cyo);

    float get_cell(int x, int y);
    void set_cell(int x, int y, float v);

    void decay_cells(float fac);
    void clear_cells();

    bool check_line_clear(Vec2f pos1, Vec2f pos2, float thresh);
    bool check_bounds(const int x, const int y) const;
};

class NavController {
public: // lol fuck it
    Grid grid;

    Vec2i dest_pos;
    Vec2f cur_pos;

    int path_gen_count; // how many times have we calculated a new path

    std::vector<Vec2i> path_waypoints;
    int cur_waypt_index;

    // aaaaa
    void do_recalculate_path();
    bool do_validate_path();

    NavController(Vec2i);

    /**
     * Updates the current position of the navigator.  Returns true if we
     * should target a new waypoint as the next step of the path.
     */
    bool update_cur_pos(Vec2f);
    
    /**
     * Gets the current position.  Nothing unusual.
     */
    Vec2f get_current_pos();

    /**
     * Updates a cell, increasing or decreasing its weight.  Does not update
     * the current route, must call revalidate_path() for that.
     */
    void update_cell(Vec2i, bool);

    /**
     * Checks to see if the current path is still valid, ie. that it doesn't
     * intersect any walls.  If this returns false, then the current path was
     * invalid and we computed a new one and you should start heading towards
     * the result of get_current_target() now.
     */
    bool validate_path();

    /**
     * Returns the current target, if there is one.
     */
    Vec2i get_current_target();

    /**
     * Returns if we're close to the destination.  Should probably stop
     * navigating if this is true.
     */
    bool is_done();
    
    /**
     * Borrows the internal grid.  Don't modify it or try to use it after
     * freeing this.
     */
    Grid* get_grid();
    
    /**
     * Borrows the internal grid.  Don't modify it or try to use it after
     * freeing this.
     */
    std::vector<Vec2i>* get_path();
};

#endif

