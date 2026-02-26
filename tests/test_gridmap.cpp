#include <gridmap.h>
#include <iostream>
#include <cassert>
#include <random>
#include <data_models.h>
// Set this to a location where test-generated gridmap output files can be written.
// In C++, unlike Python's os.makedirs, you need to ensure the directory exists or create it yourself.
// Example: Create folder "test_output" in current directory.
#include <filesystem>
std::string const GRIDMAP_TEST_OUTPUT_PATH = "./tests/test_outputs/gridmap_tests";

// Ensure the directory exists at the start of main (or before tests run)
struct EnsureTestOutputDir {
    EnsureTestOutputDir() {
        std::filesystem::create_directories(GRIDMAP_TEST_OUTPUT_PATH);
    }
} ensureTestOutputDirInstance;


bool test_gridmap_coordinates_constructor() {
    // Test with y_bound_upper = 5, y_bound_lower = -5, x_bound_upper = 10, x_bound_lower = -10
    gridmap_bounds bounds = {5, -5, 10, -10}; // 10m x 20m grid
    double resolution = 1; // 1m resolution
    GridMapCoordinates gridmap_coords(bounds, resolution);
    
    // Test the width and height in cells
    assert(gridmap_coords.get_width_cells() == 10);
    assert(gridmap_coords.get_height_cells() == 20);

    // Test Resolution
    assert(gridmap_coords.get_resolution() == resolution);

    // Test the bounds
    assert(gridmap_coords.get_bounds().y_bound_upper == bounds.y_bound_upper);
    assert(gridmap_coords.get_bounds().y_bound_lower == bounds.y_bound_lower);
    assert(gridmap_coords.get_bounds().x_bound_upper == bounds.x_bound_upper);
    assert(gridmap_coords.get_bounds().x_bound_lower == bounds.x_bound_lower);

    std::cout << "GridMapCoordinates constructor test passed" << std::endl;
    return true;
}

bool test_gridmap_coordinates_xy_to_vu() {
    // Test with y_bound_upper = 5, y_bound_lower = -5, x_bound_upper = 10, x_bound_lower = -10
    gridmap_bounds bounds = {5, -5, 10, -10}; // 10m x 20m grid
    double resolution = 1; // 1m resolution
    GridMapCoordinates gridmap_coords(bounds, resolution);

    // Test with point (0, 0)
    point2 p0 = {0, 0};
    std::optional<point_uv> uv = gridmap_coords.xy_to_uv(p0);
    assert(uv);
    assert(uv->u == 5);
    assert(uv->v == 10);

    // Test with point (10, 10)
    point2 p1 = {10, 0};
    std::optional<point_uv> uv2 = gridmap_coords.xy_to_uv(p1);
    assert(uv2);
    assert(uv2->u == 5);
    assert(uv2->v == 0);  

    // Test with point (0, 10)
    point2 p2 = {0, 5};
    std::optional<point_uv> uv3 = gridmap_coords.xy_to_uv(p2);
    assert(uv3);
    assert(uv3->u == 0);
    assert(uv3->v == 10);

    // Test w/ out of bounds point
    point2 p3 = {10, 10};
    std::optional<point_uv> uv4 = gridmap_coords.xy_to_uv(p3);
    assert(!uv4);

    std::cout << "GridMapCoordinates xy to vu test passed" << std::endl;
    return true;
}

bool test_gridmap_coordinates_vu_to_xy() {
    // Test with y_bound_upper = 5, y_bound_lower = -5, x_bound_upper = 10, x_bound_lower = -10
    gridmap_bounds bounds = {5, -5, 10, -10}; // 10m x 20m grid
    double resolution = 1; // 1m resolution
    GridMapCoordinates gridmap_coords(bounds, resolution);
    point_uv point = {4, 9};
    point2 xy = gridmap_coords.uv_to_xy(point);
    std::cout << "xy: " << xy.x << ", " << xy.y << std::endl;
    assert(xy.x == 0 + resolution / 2);
    assert(xy.y == 0 + resolution / 2);

    // Test with point (10, 0)
    point_uv point_uv = {4, 19};
    point2 xy2 = gridmap_coords.uv_to_xy(point_uv);
    assert(xy2.y == 0 + resolution / 2);
    assert(xy2.x == -10 + resolution / 2);

    std::cout << "GridMapCoordinates vu to xy test passed" << std::endl;
    return true;
}

bool test_dense_grid_layer_constructor() {
    // Test with y_bound_upper = 5, y_bound_lower = -5, x_bound_upper = 10, x_bound_lower = -10
    gridmap_bounds bounds = {5, -5, 10, -10}; // 10m x 20m grid
    double resolution = 1; // 1m resolution
    GridMapCoordinates gridmap_coords(bounds, resolution);
    DenseGridLayer dense_grid_layer(gridmap_coords);
    assert(dense_grid_layer.get_data().size() == gridmap_coords.get_width_cells() * gridmap_coords.get_height_cells());
    std::cout << "DenseGridLayer constructor test passed" << std::endl;
    return true;
}

bool test_dense_grid_layer_uv_to_vec_idx() {
    // Test with y_bound_upper = 5, y_bound_lower = -5, x_bound_upper = 10, x_bound_lower = -10
    gridmap_bounds bounds = {5, -5, 10, -10}; // 10m x 20m grid
    double resolution = 1; // 1m resolution
    GridMapCoordinates gridmap_coords(bounds, resolution);
    DenseGridLayer dense_grid_layer(gridmap_coords);
    assert(dense_grid_layer.uv_to_vec_idx({4, 0}) ==  0 * gridmap_coords.get_width_cells() + 4);
    assert(dense_grid_layer.uv_to_vec_idx({6, 2}) ==  2 * gridmap_coords.get_width_cells() + 6);
    std::cout << "DenseGridLayer uv to vec idx test passed" << std::endl;
    return true;
}

bool test_dense_grid_layer_increment_clear_grid() {
    // Test with y_bound_upper = 5, y_bound_lower = -5, x_bound_upper = 10, x_bound_lower = -10
    gridmap_bounds bounds = {5, -5, 10, -10}; // 10m x 20m grid
    double resolution = 1; // 1m resolution
    GridMapCoordinates gridmap_coords(bounds, resolution);
    DenseGridLayer dense_grid_layer(gridmap_coords);
    dense_grid_layer.increment_point({4, 9}, 1);
    assert(dense_grid_layer.get_point({4, 9}) == 1);
    dense_grid_layer.increment_point({4, 9}, 1);
    assert(dense_grid_layer.get_point({4, 9}) == 2);
    dense_grid_layer.clear_grid();
    assert(dense_grid_layer.get_point({4, 9}) == 0);
    std::cout << "DenseGridLayer increment and clear grid test passed" << std::endl;
    return true;
}

bool test_dense_grid_layer_visualize() {
    gridmap_bounds bounds = {10, -10, 10, -10};
    double resolution = 0.1;
    GridMapCoordinates coords = GridMapCoordinates(bounds, resolution);
    DenseGridLayer gridmap = DenseGridLayer(coords);

    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 5.0);
    for (int i = 0; i < 1000000; ++i) {
        point2 pt{dist(rng), dist(rng)};
        auto uv = coords.xy_to_uv(pt);
        if (uv) gridmap.increment_point(*uv);
    }

    gridmap.visualize_grid(GRIDMAP_TEST_OUTPUT_PATH + "/test_dense_grid_layer_visualize.pgm");
    return true;
}

int main() {
    if (!test_gridmap_coordinates_constructor()) {
        std::cout << "Test gridmap coordinates constructor failed" << std::endl;
        return 1;
    }
    if (!test_gridmap_coordinates_xy_to_vu()) {
        std::cout << "Test gridmap coordinates xy to vu failed" << std::endl;
        return 1;
    }
    if (!test_gridmap_coordinates_vu_to_xy()) {
        std::cout << "Test gridmap coordinates vu to xy failed" << std::endl;
        return 1;
    }
    if (!test_dense_grid_layer_constructor()) {
        std::cout << "Test dense grid layer constructor failed" << std::endl;
        return 1;
    }
    if (!test_dense_grid_layer_increment_clear_grid()) {
        std::cout << "Test dense grid layer increment point failed" << std::endl;
        return 1;
    }
    if (!test_dense_grid_layer_uv_to_vec_idx()) {
        std::cout << "Test dense grid layer uv to vec idx failed" << std::endl;
        return 1;
    }
    if (!test_dense_grid_layer_visualize()) {
        std::cout << "Test dense grid layer visualize failed" << std::endl;
        return 1;
    }
    std::cout << "All tests passed" << std::endl;
    return 0;
}


