#include <path_finder.h>
#include <cassert>
#include <queue>
#include <vector>
#include <data_models.h>
#include <unordered_map>

/** Base PathFinder Class Implementations */

/** Returns uv indicies of the inbounds & valid neighbors of passed point. */
std::vector<point_uv> PathFinder::get_valid_neighbors(point_uv p, bool diags) {
    std::vector<point_uv> candidate_neighbors = {
        point_uv{p.u + 1, p.v},
        point_uv{p.u - 1, p.v},
        point_uv{p.u, p.v + 1},
        point_uv{p.u, p.v - 1}
    };

    if (diags) {
        candidate_neighbors.push_back(point_uv{p.u + 1, p.v + 1});
        candidate_neighbors.push_back(point_uv{p.u + 1, p.v - 1});
        candidate_neighbors.push_back(point_uv{p.u - 1, p.v + 1});
        candidate_neighbors.push_back(point_uv{p.u - 1, p.v - 1});
    }

    std::vector<point_uv> neighbors;
    for (auto const& uv_pt : candidate_neighbors) {
        if (this->in_bounds(uv_pt) && this->cell_free(uv_pt)) {
            neighbors.push_back(uv_pt);
        }
    }
    return neighbors;
}

std::optional<std::vector<point_uv>> BFS::find_path(point_uv start, point_uv end, bool diags) {
    std::queue<point_uv> frontier;
    std::unordered_map<int, int> came_from;
    int start_idx = this->uv_to_vec(start);
    came_from[start_idx] = start_idx;  // avoid re-pushing start when visiting its neighbors

    frontier.push(start);

    while (!frontier.empty()) {

        point_uv current = frontier.front();

        if (current == end) { // if we've reached goal
            std::vector<int> path = reconstruct_path(this->uv_to_vec(end), this->uv_to_vec(start), came_from);
            return std::optional<std::vector<point_uv>>(this->vec_to_uv(path));
        } else {
            int curr_idx = this->uv_to_vec(current);
            for (auto const& nbr : this->get_valid_neighbors(current, diags)) {
                auto[it, inserted] = came_from.insert({this->uv_to_vec(nbr), this->uv_to_vec(current)});
                if (inserted) { frontier.push(nbr); }
            }
        frontier.pop();
        }
    }
    
    return std::nullopt; // no path found if we reach this point
}

/** Satisfies base interface; uses default heuristic weight 1.0. */
std::optional<std::vector<point_uv>> AStarPathFinder::find_path(point_uv start, point_uv end) {
    std::vector<point_uv> frontier;
    std::vector<point_uv> came_from;

    frontier.push_back(start);

    // TODO: Implement A* pathfinding algorithm
    return std::vector<point_uv>();

}

/** Same with explicit heuristic weight (and any other A*-specific options). */
std::optional<std::vector<point_uv>> AStarPathFinder::find_path(point_uv start, point_uv end, double heuristic_weight, std::string dist_metric) {
    // TODO: Implement A* Pathfinding Algorithm
    return std::vector<point_uv>();
}

/** Utils Functions */
std::vector<int> reconstruct_path(int end, int start, std::unordered_map<int, int> const& mappings) {
    int current = end;
    std::vector<int> path;
    path.push_back(end);

    while (current != start) {
        auto it = mappings.find(current);
        if (it == mappings.end()) {
            assert(false && "came_from missing node; path reconstruction broken");
            return std::vector<int> ();
        }
        auto [key, parent] = *it;
        current = parent;
        path.push_back(parent);
    }
    std::reverse(path.begin(), path.end()); // reverse so that start is at index 0
    return path;
}