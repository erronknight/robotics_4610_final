#include "nav.hxx"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include "common.hxx"

using std::cout;
using std::endl;

Vec2i::Vec2i() : x(0), y(0) {
    // nothing else
}

Vec2i::Vec2i(int x, int y) : x(x), y(y) {
    // nothing else
}

Vec2f Vec2i::to_vec2f() const {
    return Vec2f((float) x, (float) y);
}

Vec2f::Vec2f() : x(0), y(0) {
    // nothing else
}

Vec2f::Vec2f(float x, float y) : x(x), y(y) {
    // nothing else
}

Vec2i Vec2f::to_vec2i() const {
    return Vec2i((int) roundf(x), (int) roundf(y));
}

//bool operator==(const Vec2i& lhs, const Vec2i& rhs) {
//    return lhs.x == rhs.x && lhs.y == rhs.y;
//}

static float calc_pos_dist(Vec2i a, Vec2i b) {
    return distf(a.x, a.y, b.x, b.y);
}

static float calc_posf_dist(Vec2f a, Vec2f b) {
    return distf(a.x, a.y, b.x, b.y);
}

static float calc_posfi_dist(Vec2f a, Vec2i b) {
    return calc_posf_dist(a, Vec2f((float) b.x, (float) b.y));
}

class Vec2i_Hash {
public:
    size_t operator()(const Vec2i& p) const {
        // Two random ~31-bit primes, that's how this works, right?
        int p1 = 2031567413;
        int p2 = 390591239;
        return p.x * p1 + p.y * p2;
    }
};

Grid::Grid(int w, int h, int cxo, int cyo) {
    width = w;
    height = h;
    cx_off = cxo;
    cy_off = cyo;
    data.resize(w * h, 0.0f);
}

static int calc_cell_off(Grid* g, int x, int y) {
    int rx = x + g->cx_off;
    int ry = y + g->cy_off;
    if (rx < 0 || rx >= g->width || ry < 0 || ry >= g->height) {
        return -1;
    }

    return rx * g->width + ry;
}

float Grid::get_cell(int x, int y) {
    int i = calc_cell_off((Grid*) this, x, y);
    if (i == -1) {
        return NAN;
    }

    return data[i];
}

void Grid::set_cell(int x, int y, float v) {
    int i = calc_cell_off((Grid*) this, x, y);
    if (i == -1) {
        return;
    }
    
    data[i] = v;
}

void Grid::decay_cells(float fac) {
    for (size_t i = 0; i < data.size(); i++) {
        data[i] *= fac;
    }
}

void Grid::clear_cells() {
    decay_cells(0.0f);
}

bool Grid::check_line_clear(Vec2f pos1, Vec2f pos2, float thresh) {
    float dist = calc_posf_dist(pos1, pos2);
    float steps = roundf(dist / 0.7); // eyeballed number

    // Check start and finish as edge cases.
    auto p1i = pos1.to_vec2i();
    auto p2i = pos2.to_vec2i();
    if (get_cell(p1i.x, p1i.y) >= thresh || get_cell(p2i.x, p2i.y) >= thresh) {
        return false;
    }

    // Now step through the line and see if they're all clear.
    float sdx = ((float) (pos2.x - pos1.x)) / steps;
    float sdy = ((float) (pos2.y - pos1.y)) / steps;
    int lx = INT_MAX;
    int ly = INT_MAX;
    for (int i = 1; i < steps; i++) {
        // I wonder if we should floor and ceil it and do 4 checks or something.
        int sx = (int) roundf(pos1.x + i * sdx);
        int sy = (int) roundf(pos1.y + i * sdy);

        // Quick check to avoid having to do more math.        
        if (sx == lx && sy == ly) {
            continue;
        }

        if (get_cell(sx, sy) >= thresh) {
            return false;
        }

        lx = sx;
        ly = sy;
    }

    return true;
}

bool Grid::check_bounds(const int x, const int y) const {
    int rx = x + cx_off;
    int ry = y + cy_off;
    return rx > 0 && rx <= width && ry > 0 && ry <= height;
}

static bool find_nearest_empty_cell(Vec2i src, int radius, Grid* grid, Vec2i* retp) {
    float best_dist = -1;
    for (int dx = -radius; dx <= radius; dx++) {
        for (int dy = -radius; dy <= radius; dy++) {
            Vec2i ckpos(src.x + dx, src.y + dy);
            float pw = grid->get_cell(ckpos.x, ckpos.y);
            if (pw < BASE_OCCUPIED_THRESH) {
                float dist = calc_pos_dist(ckpos, src);
                if (best_dist < 0 || dist < best_dist) {
                    *retp = ckpos;
                    best_dist = dist;
                }
            }
        }
    }
    return best_dist >= -0.5;
}

static float calc_astar_heuristic(Vec2i pos, Vec2i startpos, Vec2i endpos) {
    return calc_pos_dist(pos, startpos) + calc_pos_dist(pos, endpos);
}

static std::vector<Vec2i> astar_reconstruct_path(
    std::unordered_map<Vec2i, Vec2i, Vec2i_Hash>& links, Vec2i last) {
    
    std::vector<Vec2i> path;
    path.push_back(last);
    
    Vec2i cur = last;
    while (links.find(cur) != links.end()) {
        cur = links[cur];
        path.push_back(cur);
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

//using cell_score_t = std::pair<Vec2i, float>;
//using cell_score_cont_t = std::vector<cell_score_t>;
using cell_weight_map = std::unordered_map<Vec2i, float, Vec2i_Hash>;

static inline bool astar_fn_check_cell(Vec2i p, cell_weight_map& wc, float thresh) {
    Vec2i ckpos;
    #define ckweightok(dx, dy) \
    ckpos = Vec2i(p.x + dx, p.y + dy); \
    { \
        auto it = wc.find(ckpos); \
        if (it == wc.end() || it->second >= thresh) return false; \
    }
    ckweightok(0, 0);
    #undef ckweightok
    return true;
}

/**
 * Fills the vec with geometric neighbors.  Only checks bounds, does not
 * account for the weights of the cells.
 */
static void astar_find_neighbors(Vec2i pos, Grid* grid, std::vector<Vec2i>& out, float thresh) {
    // Find the weights of the cells in our general proximity.
    cell_weight_map weight_cache;
    int cnt = 0;
    for (int dx = -2; dx <= 2; dx++) {
        for (int dy = -2; dy <= 2; dy++) {
            Vec2i p(pos.x + dx, pos.y + dy);
            if (grid->check_bounds(p.x, p.y)) {
                weight_cache[p] = grid->get_cell(p.x, p.y);
                //cout << "AT: " << p.x << "," << p.y << " weight " << weight_cache[p] << endl;
                cnt++;
            }
        }
    }
    //cout << "got " << cnt << " ok cells" << endl;

    Vec2i npos;
    #define tryaddneighbor(dx, dy) \
    npos = Vec2i(pos.x + dx, pos.y + dy); \
    if (astar_fn_check_cell(npos, weight_cache, thresh)) out.push_back(npos);
    tryaddneighbor(-1, 0);
    tryaddneighbor(0, -1);
    tryaddneighbor(0, 1);
    tryaddneighbor(1, 0);
    #undef tryaddneighbor
}

static std::vector<Vec2i> astar_compute_path(Vec2i startpos, Vec2i endpos, Grid* grid) {
    // Init stuff.
    std::unordered_map<Vec2i, Vec2i, Vec2i_Hash> came_from;
    cell_weight_map g_scores;
    cell_weight_map f_scores;

    // Janky thing to make our priority queue work.
    auto comparator =
        [&](const Vec2i& e1, const Vec2i& e2) {
            // This might be bad, idk.
            return f_scores[e1] > f_scores[e2];
        };
    std::priority_queue<
        Vec2i, std::vector<Vec2i>, decltype(comparator)> open_set(comparator);
    // Other janky thing for keeping track of what's in open_set.
    std::unordered_set<Vec2i, Vec2i_Hash> os_cont;

    float start_heur = calc_astar_heuristic(startpos, startpos, endpos);
    open_set.push(startpos);
    os_cont.insert(startpos);
    g_scores[startpos] = 0;
    f_scores[startpos] = start_heur;

    // Temporary cache used on each iteration.
    std::vector<Vec2i> cur_neighbors;
    cur_neighbors.reserve(8);

    // Main A* pathfinding loop.
    while (open_set.size() > 0) {
        Vec2i cur_best = open_set.top();

        // If we're at the end, then reconstruct the path and exit.
        if (cur_best == endpos) {
            return astar_reconstruct_path(came_from, cur_best);
        }

        // Remove cur_best
        auto it = os_cont.find(cur_best);
        if (it != os_cont.end()) {
            os_cont.erase(it);
            open_set.pop(); // do it in here to keep consistency
        } else {
            // TODO Figure out what to do here.
            cout << "ASTAR ERROR: couldnt remove thing but we thought we had to" << endl;
        }

        // This does all the work of finding the valid neighbors.
        astar_find_neighbors(cur_best, grid, cur_neighbors, BASE_OCCUPIED_THRESH);
        //cout << "NAV DEBUG: " << cur_best.x << "," << cur_best.y << " has " << cur_neighbors.size() << " neighbors" << endl;
        for (Vec2i neigh : cur_neighbors) {
            float cell_score = grid->get_cell(neigh.x, neigh.y);

            float neigh_dist = calc_pos_dist(cur_best, neigh);
            float tentative_gscore = g_scores[cur_best] + neigh_dist;
            if (g_scores.find(neigh) == g_scores.end() || tentative_gscore < g_scores[neigh]) {
                float neigh_heur = calc_astar_heuristic(neigh, startpos, endpos);
                float f_score = tentative_gscore + neigh_heur;
                came_from[neigh] = cur_best;
                g_scores[neigh] = tentative_gscore;
                f_scores[neigh] = f_score;

                // FIXME See if we can make this less bad.
                auto it = os_cont.find(neigh);
                if (it == os_cont.end()) {
                    open_set.push(neigh);
                    os_cont.insert(neigh);
                } else {
                    // In this case we have to remove and re-add it.
                    //cout << "ASTAR ERROR: needed to add thing but there was already thing" << endl;
                }

            }

        }
        
        // Clean up for the next iteration.
        cur_neighbors.clear();
    }

    // We didn't get anywhere, so return an empty list, which implies that
    // there is no path we can take.
    return std::vector<Vec2i>();
}

std::vector<Vec2i> simplify_path(std::vector<Vec2i> orig, Grid* grid) {
    if (orig.size() == 0) {
        return orig;
    }
    
    std::vector<Vec2i> simplified;
    simplified.push_back(orig[0]);
    
    Vec2i step_start = orig[0];
    for (int i = 1; i < orig.size(); i++) {
        Vec2i next_end = orig[i];
        
        // If this cell isn't a straight shot from the last start, then make a new leg.
        if (!grid->check_line_clear(step_start.to_vec2f(), next_end.to_vec2f(), BASE_OCCUPIED_THRESH)) {
            simplified.push_back(next_end);
            step_start = next_end;
        }
    }

    // After we're done with the list then make sure we get to the dest.
    // FIXME Weird conditional because operator overloading.
    if (!(simplified.back() == orig.back())) {
        simplified.push_back(orig.back());
    }

    return simplified;
}

/**
 * Because of rounding errors we might end up being closer to the next step in
 * our path than the first step, for some reason.  This finds where in the path
 * we're *actually* closest to so we can target the waypoint for that step
 * instead of just blindly targetting the first waypoint.
 */
int find_best_target_step(Vec2f pos, NavController* nav) {

    Vec2i src = nav->path_waypoints[0];

    // Assume the first one is correct, just for now.
    int best_i = 0;
    float best_dist = calc_posfi_dist(pos, src);

    for (int i = 1; i < nav->path_waypoints.size(); i++) {
        Vec2i dest = nav->path_waypoints[i];

        float run_dist = calc_pos_dist(src, dest);
        int steps = (int) roundf(run_dist / 0.5); // eyeballed

        // Now step through the line and see if they're all clear.
        float sdx = ((float) (dest.x - src.x)) / ((float) steps);
        float sdy = ((float) (dest.y - src.y)) / ((float) steps);
        for (int i = 1; i < steps; i++) {
            // I wonder if we should floor and ceil it and do 4 checks or something.
            int sx = roundf(src.x + i * sdx);
            int sy = roundf(src.y + i * sdy);

            float sd = distf(sx, sy, pos.x, pos.y);
            if (sd < best_dist) {
                best_i = 0;
                best_dist = sd;
            }
        }

    }

    return best_i;
}

void NavController::do_recalculate_path() {
    // Get rid of old path, get ready for new navigation.
    path_gen_count++;
    path_waypoints.clear();
    cur_waypt_index = 0;

    Vec2i start_pos;
    auto cpi = cur_pos.to_vec2i();
    bool start_ok = find_nearest_empty_cell(cpi, 3, &grid, &start_pos);
    if (!start_ok) {
        cout << "NAV ERROR: couldn't find satisfactory start pos, using true pos" << endl;
        start_pos = cpi;
    }

    std::vector<Vec2i> path = astar_compute_path(start_pos, dest_pos, &grid);

    // TODO If we don't find a path then decay things?
    path_waypoints = simplify_path(path, &grid);
    
    int best_step = find_best_target_step(cur_pos, this);
    if (best_step > 0) {
        cout << "NAV: best start target for new route is " << best_step << endl;
    }
    cur_waypt_index = best_step;

}

bool NavController::do_validate_path() {
    Vec2f at = cur_pos;

    // Go through all the points and make sure that the line between them is
    // not occupied.
    for (int i = cur_waypt_index; i < path_waypoints.size(); i++) {
        Vec2i waypt = path_waypoints[i];
        Vec2f wayptf = waypt.to_vec2f();
        bool clear = grid.check_line_clear(at, wayptf, BASE_OCCUPIED_THRESH);
        if (!clear) {
            return false;
        }
        at = wayptf;
    }

    return true;
}

NavController::NavController(Vec2i dest) :
    grid(MAP_WIDTH, MAP_HEIGHT, MAP_WIDTH / 2, MAP_HEIGHT / 2),
    dest_pos(dest) {

    // For now, just target the destination.
    path_gen_count = 0;
    cur_waypt_index = 0;
    path_waypoints.push_back(dest_pos);
}

bool NavController::update_cur_pos(Vec2f pos) {
    cur_pos = pos;
    if (is_done()) {
        return false;
    }

    // If we're near to the current waypoint, then go onto the next one.
    if (calc_posfi_dist(cur_pos, get_current_target()) < STEP_PROX_THRESH) {
        // But only if we're not at the end.
        if (cur_waypt_index < path_waypoints.size() - 1) {
            cur_waypt_index++;
            return true;
        }
    }

    return false;
}

Vec2f NavController::get_current_pos() {
    return cur_pos;
}

void NavController::update_cell(Vec2i cell, bool chg) {
    float cur_val = grid.get_cell(cell.x, cell.y);
    // TODO Tune these numbers.
    float next_val = clampf(cur_val + CELL_HIT_DIFF * (chg ? 1 : -1), 0, 10);
    grid.set_cell(cell.x, cell.y, next_val);
}

bool NavController::validate_path() {
    if (!this->do_validate_path()) {
        this->do_recalculate_path();
        return false;
    }
    
    // Anything else?  I feel like maybe.
    return true;
}

Vec2i NavController::get_current_target() {
    return path_waypoints[cur_waypt_index];
}

bool NavController::is_done() {
    return calc_posfi_dist(cur_pos, dest_pos) < STEP_PROX_THRESH;
}

Grid* NavController::get_grid() {
    return &grid;
}

std::vector<Vec2i>* NavController::get_path() {
    return &path_waypoints;
}

