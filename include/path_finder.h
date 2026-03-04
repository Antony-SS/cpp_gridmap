#pragma once

#include <gridmap.h>
#include <vector>
#include <cassert>
#include <data_models.h>

class PathFinder {
public:
    PathFinder(std::vector<int> const& data, int grid_height, int grid_width) : data(data), grid_height(grid_height), grid_width(grid_width)  {}

    virtual std::optional<std::vector<point_uv>> find_path(point_uv start, point_uv end, bool diags) = 0;

    /* Index into flat vector row-major form. */
    int uv_to_vec(point_uv p) const { return p.v * this->get_height() + p.u; }

    /* Version for a vector of point_uv */
    std::vector<int> uv_to_vec(std::vector<point_uv> const& points) const {
        std::vector<int> indices;
        indices.reserve(points.size());
        for (const auto& p : points) {
            indices.push_back(uv_to_vec(p));
        }
        return indices;
    }

    /** Convert vector index to uv coordinates */
    point_uv vec_to_uv(int idx) const { 
        return point_uv{
            idx % this->grid_width,
            static_cast<int> (idx) / this->grid_width
        };
    }

    /** Version for a vector of indices */
    std::vector<point_uv> vec_to_uv(std::vector<int> const& indices) const {
        std::vector<point_uv> points;
        points.reserve(indices.size());
        for (const auto& idx : indices) {
            points.push_back(vec_to_uv(idx));
        }
        return points;
    }

    std::vector<point_uv> get_valid_neighbors(point_uv p, bool diags = false);

    const int get_height() const { return this->grid_height; }
    const int get_width() const { return this->grid_width; }

protected:
    const std::vector<int> data;
    const int grid_height;
    const int grid_width;

    /** Check passed point is inbounds */
    const bool in_bounds(point_uv const& p) {
        return ((p.u >= 0 && p.u <= this->grid_width - 1)
                && (p.v >= 0 && p.v <= this->grid_height - 1));
    }

    /** Check cell isn't defined as an obstacle (-1) */
    bool cell_free(point_uv const& p) const {
        return this->data.at(this->uv_to_vec(p)) != -1;
    }
};

class BFS : public PathFinder {
    /** Constructor */

    public:
        BFS(std::vector<int> const& data, int grid_height, int grid_width): PathFinder(data, grid_height, grid_width) {}

        /** Implement base find path, just calls find path with filled in params */
        std::optional<std::vector<point_uv>> find_path(point_uv start, point_uv end) { return find_path(start, end, false); }

        /** Implement find path with optional diagonal movement */
        std::optional<std::vector<point_uv>> find_path(point_uv start, point_uv end, bool diags) override;
};

class Dijkstra : public PathFinder {
public:
    Dijkstra(std::vector<int> const& data, int grid_height, int grid_width): PathFinder(data, grid_height, grid_width) {}

    /** For lazy calling */
    std::optional<std::vector<point_uv>> find_path(point_uv start, point_uv end) { return find_path(start, end, false); }

    /** Implement find path with optional diagonal movement */
    std::optional<std::vector<point_uv>> find_path(point_uv start, point_uv end, bool diags) override;
};

class AStarPathFinder : public PathFinder {
public:
    /** Constructor for AStarPathFinder, doesn't have state beyond base class PathFinder */
    AStarPathFinder(std::vector<int> const& data, int grid_height, int grid_width): PathFinder(data, grid_height, grid_width) {}

    /** Satisfies base interface; uses default heuristic weight 1.0. */
    std::optional<std::vector<point_uv>> find_path(point_uv start, point_uv end, bool diags) override;

    /** Same with explicit heuristic weight (and any other A*-specific options). */
    std::optional<std::vector<point_uv>> find_path(point_uv start, point_uv end, double heuristic_weight, std::string dist_metric = "euclidean");
};

/** Utils Functions */

/** Reconstructs path from the 'came_from' mapping, which assumes */
std::vector<int> reconstruct_path(int end, int start, std::unordered_map<int, int> const& mappings);