#include <gridmap.h>
#include <optional>
#include <vector>
#include <cassert>


GridMapCoordinates::GridMapCoordinates(gridmap_bounds bounds, double resolution)
    : bounds(bounds), resolution(resolution) {
        assert(resolution > 0 && "resolution must be postive");
        assert(bounds.x_bound_upper > bounds.x_bound_lower && "Upper X bound must be greater than lower X bound");
        assert(bounds.y_bound_upper > bounds.y_bound_lower && "Upper Y bound must be greater than lower Y bound");
    }

std::optional<point_uv> GridMapCoordinates::xy_to_vu(point_xy const& point) const {
    if (point.x > bounds.x_bound_upper || point.x < bounds.x_bound_lower
            || point.y > bounds.y_bound_upper || point.y < bounds.y_bound_lower) {
                return std::nullopt;
            }
    
    int point_v = static_cast<int>(std::floor(bounds.x_bound_upper - point.x) / resolution); // very clean way to write this coordinate conversion, v grows as point.x decreases, which is what we want
    int point_u = static_cast<int>(std::floor(bounds.y_bound_upper - point.y) / resolution);
    return point_uv{point_u, point_v};

}

point_xy GridMapCoordinates::vu_to_xy(point_uv const& point) const {
    assert(point.u > 0 && point.u < this->get_width_cells() && point.v > 0 && point.v < this->get_height_cells() && "UV coords accessed are out of bounds!");
    double point_x =  (bounds.x_bound_upper - point.v * resolution) - resolution / 2;
    double point_y =  (bounds.y_bound_upper - point.u * resolution) - resolution / 2;
    return point_xy{point_x, point_y};
}

std::vector<point_uv> GridMapCoordinates::xy_to_vu(std::vector<point_xy> const& points) const {
    std::vector<point_uv> uv_points;
    for (size_t i = 0; i < points.size(); i++) {
        std::optional<point_uv> point_uv = xy_to_vu(points.at(i));
        if (point_uv) {
            uv_points.push_back(*point_uv);
        }
    }
    return uv_points;
}

std::vector<point_xy> GridMapCoordinates::vu_to_xy(std::vector<point_uv> const& points) const {
    std::vector<point_xy> xy_points;
    for (size_t i = 0; i < points.size(); i++) {
        xy_points.push_back(vu_to_xy(points.at(i)));
    }
    return xy_points;
}
