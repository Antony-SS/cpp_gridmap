#include <gridmap.h>
#include <optional>
#include <vector>
#include <cassert>
#include <fstream>
#include <data_models.h>


GridMapCoordinates::GridMapCoordinates(gridmap_bounds bounds, double resolution)
    : bounds(bounds), resolution(resolution) {
        assert(resolution > 0 && "resolution must be postive");
        assert(bounds.x_bound_upper > bounds.x_bound_lower && "Upper X bound must be greater than lower X bound");
        assert(bounds.y_bound_upper > bounds.y_bound_lower && "Upper Y bound must be greater than lower Y bound");
    }

std::optional<point_uv> GridMapCoordinates::xy_to_uv(point2 const& point) const {
    if (point.x > bounds.x_bound_upper || point.x < bounds.x_bound_lower
            || point.y > bounds.y_bound_upper || point.y < bounds.y_bound_lower) {
                return std::nullopt;
            }
    
    int point_v = static_cast<int>(std::floor((bounds.x_bound_upper - point.x) / resolution));
    int point_u = static_cast<int>(std::floor((bounds.y_bound_upper - point.y) / resolution));
    return point_uv{point_u, point_v};

}

point2 GridMapCoordinates::uv_to_xy(point_uv const& point) const {
    assert(point.u > 0 && point.u < this->get_width_cells() && point.v > 0 && point.v < this->get_height_cells() && "UV coords accessed are out of bounds!");
    double point_x =  (bounds.x_bound_upper - point.v * resolution) - resolution / 2;
    double point_y =  (bounds.y_bound_upper - point.u * resolution) - resolution / 2;
    return point2{point_x, point_y};
}

std::vector<point_uv> GridMapCoordinates::xy_to_uv(std::vector<point2> const& points) const {
    std::vector<point_uv> uv_points;
    for (size_t i = 0; i < points.size(); i++) {
        std::optional<point_uv> point_uv = xy_to_uv(points.at(i));
        if (point_uv) {
            uv_points.push_back(*point_uv);
        }
    }
    return uv_points;
}

std::vector<point2> GridMapCoordinates::uv_to_xy(std::vector<point_uv> const& points) const {
    std::vector<point2> xy_points;
    for (size_t i = 0; i < points.size(); i++) {
        xy_points.push_back(uv_to_xy(points.at(i)));
    }
    return xy_points;
}

DenseGridLayer::DenseGridLayer(GridMapCoordinates gridmap_coords)
    : gridmap_coords(gridmap_coords) {
        data = std::vector<int>(this->gridmap_coords.get_width_cells() * this->gridmap_coords.get_height_cells(), 0);
    }

DenseGridLayer::DenseGridLayer(GridMapCoordinates gridmap_coords, std::vector<int> data)
    : gridmap_coords(gridmap_coords), data(std::move(data)) {
        assert(this->data.size() == this->gridmap_coords.get_height_cells() * this->gridmap_coords.get_width_cells());
    }


void DenseGridLayer::increment_point(point_uv const& point, int weight) {
    assert(point.u >= 0 && point.u < this->gridmap_coords.get_width_cells()
                && point.v >= 0 && point.v < this->gridmap_coords.get_height_cells());
    data.at(this->uv_to_vec_idx(point)) += weight;
}

int DenseGridLayer::get_point(point_uv const& point) const {
    assert(point.u >= 0 && point.u < this->gridmap_coords.get_width_cells()
        && point.v >= 0 && point.v < this->gridmap_coords.get_height_cells());
    return data.at(this->uv_to_vec_idx(point));
}

void DenseGridLayer::visualize_grid(std::string const& output_path, bool binary_grid) const {
    auto max_it = std::max_element(data.begin(), data.end());
    int max_val = *max_it;

    int width = this->gridmap_coords.get_width_cells();
    int height = this->gridmap_coords.get_height_cells();

    this->write_pgm(output_path, this->get_data(), width, height, max_val);
}

void DenseGridLayer::write_pgm(std::string const& filepath, std::vector<int> data, int width, int height, int max_value) const {
    std::string out_path = filepath;
    if (!(out_path.size() >= 4 && out_path.substr(out_path.size() - 4) == ".pgm")) {
        out_path += ".pgm";
    }
    std::ofstream file(out_path);
    file << "P2\n";
    file << width << " " << height << "\n";
    file << max_value << "\n";
    for (int i = 0; i < data.size(); i++) {
        if (i % width == 0 && i != 0) {
            file << "\n";
        }
        file << data.at(i) << " ";
    }
    file.close();
}



