#include <path_finder.h>
#include <cassert>
#include <queue>
#include <vector>
#include <data_models.h>
#include <unordered_map>
#include <iostream>

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
        frontier.pop(); // pop right after reading off

        if (current == end) { // if we've reached goal
            std::vector<int> path = reconstruct_path(this->uv_to_vec(end), this->uv_to_vec(start), came_from);
            return std::optional<std::vector<point_uv>>(this->vec_to_uv(path));
        } else {
            int curr_idx = this->uv_to_vec(current);
            for (auto const& nbr : this->get_valid_neighbors(current, diags)) {
                auto[it, inserted] = came_from.insert({this->uv_to_vec(nbr), this->uv_to_vec(current)});
                if (inserted) { frontier.push(nbr); }
            }
        }
    }
    return std::nullopt; // no path found if we reach this point
}

std::optional<std::vector<point_uv>> Dijkstra::find_path(point_uv start, point_uv end, bool diags) {

    std::unordered_map<int, int> came_from; // key is child, value is parent node we came from
    std::unordered_map<int, int> costs; // key is node, value is our current best stored cost to get to that point
    using entry = std::pair<int, int>; // stores <cost for node, node idx>
    auto cmp = [](const entry& a, const entry& b) { return a.first > b.first; }; // define comparator
    std::priority_queue<entry, std::vector<entry>, decltype(cmp)> frontier (cmp);
    
    came_from[this->uv_to_vec(start)] = this->uv_to_vec(start);
    costs.insert({0, this->uv_to_vec(start)});
    frontier.push({0, this->uv_to_vec(start)});

    while (!frontier.empty()) {

        auto [curr_cost_to, curr_node_idx] = frontier.top(); // read from top and pop it
        frontier.pop(); 
        // std::cout << "Current node: " << this->vec_to_uv(curr_node_idx).u << ", " << this->vec_to_uv(curr_node_idx).v << std::endl;

        if (this->vec_to_uv(curr_node_idx) == end) {
            std::vector<int> path = reconstruct_path(this->uv_to_vec(end), this->uv_to_vec(start), came_from);
            return std::optional<std::vector<point_uv>> (this->vec_to_uv(path));
        } else {
            for (auto const& nbr : this->get_valid_neighbors(this->vec_to_uv(curr_node_idx), diags)) {
                int cost_to_nbr = curr_cost_to + this->data.at(this->uv_to_vec(nbr));
                if (costs.find(this->uv_to_vec(nbr)) != costs.end()) { // already exists in map
                    if (cost_to_nbr < costs[this->uv_to_vec(nbr)]) { // if new path is cheaper
                        frontier.push({cost_to_nbr, this->uv_to_vec(nbr)});
                        costs[this->uv_to_vec(nbr)] = cost_to_nbr;
                        came_from[this->uv_to_vec(nbr)] = curr_node_idx;
                    }
                } else {  // doesn't exist in map, insert into cost dict, parents dict, and add to frontier
                    costs[this->uv_to_vec(nbr)] = cost_to_nbr;
                    came_from[this->uv_to_vec(nbr)] = curr_node_idx;
                    frontier.push({cost_to_nbr, this->uv_to_vec(nbr)});
                }
            }
        }
    }

    return std::nullopt; // we'll only reach this point if we dont find a valid path

}

/** Satisfies base interface; uses default heuristic weight 1.0. */
std::optional<std::vector<point_uv>> AStarPathFinder::find_path(point_uv start, point_uv end, bool diags) {
    std::vector<point_uv> frontier;
    std::vector<point_uv> came_from;

    frontier.push_back(start);

    // TODO: Implement A* pathfinding algorithm
    return std::optional<std::vector<point_uv>>(std::vector<point_uv>());

}

/** Same with explicit heuristic weight (and any other A*-specific options). */
std::optional<std::vector<point_uv>> AStarPathFinder::find_path(point_uv start, point_uv end, double heuristic_weight, std::string dist_metric) {
    // TODO: Implement A* Pathfinding Algorithm
    return std::optional<std::vector<point_uv>>(std::vector<point_uv>());
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