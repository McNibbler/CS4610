#ifndef VIZ_H
#define VIZ_H

#include <utility>

// New types added by Thomas, original header by Nat Tuck

// Contents inside a cell
typedef struct cell_params {
    int num_hits;
    int num_misses;
    int last_updated;
} cell_params;


// Coordinate on the occupancy map
typedef struct coord {
    int x;
    int y;

    // Overloaded equality operation
    bool operator==(const coord &other) const {
        return x == other.x && y == other.y;
    }
} coord;

// Hashing function for coord
// Courtesy of Rubens on Stackoverflow
namespace std {
    template<>
    struct hash<coord> {
        std::size_t operator()(const coord& c) const {
            using std::size_t;
            using std::hash;

            size_t hash1 = hash<int>()(c.x);
            size_t hash2 = hash<int>()(c.y);
            return hash1 ^ (hash2 << 1);
        }
    };
}

// Function instantiation
void viz_run();
int viz_hit(std::unordered_map<coord, cell_params>& occupancy_grid,
            float robot_x, float robot_y, float robot_t,
            float prev_x, float prev_y,
            float range, float angle, bool is_hit);

coord pos_to_coord(float x, float y);
std::pair<float, float> coord_to_pos(coord c);

#endif
