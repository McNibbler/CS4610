#ifndef VIZ_H
#define VIZ_H

#include <utility>

extern std::mutex mx;

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


// Node type to aid in A*
typedef struct node {
    struct coord c;
    float move_cost;
    float heur_cost;
    //node* parent_node;
    struct coord parent_node;
    
    // Overloaded comparator function
    bool operator<(const node &other) const {
        return (move_cost + heur_cost) < (other.move_cost + other.heur_cost);
    }
    // Overloaded comparator function
    bool operator>(const node &other) const {
        return (move_cost + heur_cost) > (other.move_cost + other.heur_cost);
    }
} node;


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

// Hashing function for node*
// Courtesy of Rubens on Stackoverflow
namespace std {
    template<>
    struct hash<node> {
        std::size_t operator()(const node& n) const {
            using std::size_t;
            using std::hash;

            // Hashes using the coordinate because every node has a unique one
            return hash<coord>()(n.c);
        }
    };
}

// Function instantiation
void viz_run();
int viz_hit(std::unordered_map<coord, cell_params>& occupancy_grid,
            float robot_x, float robot_y, float robot_t,
            float prev_x, float prev_y,
            float range, float angle, bool is_hit,
            std::vector<coord> path, coord end_coord, std::vector<coord> last_path);

coord pos_to_coord(float x, float y);
std::pair<float, float> coord_to_pos(coord c);
float diff(float deg1, float deg2);

#endif
