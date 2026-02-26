#include <gridmap.h>
#include <vector>
#include <cassert>


class PathFinder {
public:
    PathFinder(std::vector<int> const& data, int grid_height, int grid_width) : data(data), grid_height(grid_height), grid_width(grid_width)  {}
    
    virtual std::vector<point_uv> find_path(point_uv start, point_uv end) = 0;

    /* Index into flat vector row-major form. */
    int get_point_idx(point_uv p) const { return p.v * grid_width + p.u; }


    const int get_height() { return this->grid_height; }
    const int get_width() { return this->grid_width; }

protected:
    const std::vector<int> data;
    const int grid_height;
    const int grid_width;
};


class AStarPathFinder : public PathFinder {
public:
    AStarPathFinder(std::vector<int> const& data, int grid_height, int grid_width);

    /** Satisfies base interface; uses default heuristic weight 1.0. */
    std::vector<point_uv> find_path(point_uv start, point_uv end) override;

    /** Same with explicit heuristic weight (and any other A*-specific options). */
    std::vector<point_uv> find_path(point_uv start, point_uv end, double heuristic_weight, std::string);
private:
    // std::string distance_metric;
};