#include <path_finder.h>
#include <cassert>
#include <data_models.h>
#include <iostream>

const std::vector<int> EMPTY_TEST_GRID = {
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
};

const std::vector<int> OBSTACLE_TEST_GRID = { // to test path finding w/ obstacles
    0, 0, 0, 0, 0,
    -1, -1, -1, 0, -1,
    0, 0, 0, 0, 0,
    0, -1, -1, -1, -1,
    0, 0, 0, 0, 0,
};

const std::vector<int> WEIGHTED_TEST_GRID {
    0, 0, 0, 0, 0,
    -1, 20, -1, 1, -1,
    0, 0, 0, 0, 0,
    3, -1, -1, 15, -1,
    0, 0, 0, 0, 0,
};

const int EMPTY_TEST_GRID_HEIGHT = 5;
const int EMPTY_TEST_GRID_WIDTH = 5;
const int OBSTACLE_TEST_GRID_HEIGHT = 5;
const int OBSTACLE_TEST_GRID_WIDTH = 5;

/** Minimal concrete subclass to test PathFinder base behavior (e.g. get_valid_neighbors). */
struct PathFinderTestDouble : PathFinder {
    PathFinderTestDouble(std::vector<int> const& data, int h, int w) : PathFinder(data, h, w) {}
    std::optional<std::vector<point_uv>> find_path(point_uv, point_uv, bool diags) override { return std::nullopt; }
};

bool test_base_path_finder() {
    PathFinderTestDouble path_finder(EMPTY_TEST_GRID, EMPTY_TEST_GRID_HEIGHT, EMPTY_TEST_GRID_WIDTH);
    assert(path_finder.get_height() == EMPTY_TEST_GRID_HEIGHT);
    assert(path_finder.get_width() == EMPTY_TEST_GRID_WIDTH);
    assert(path_finder.uv_to_vec(point_uv{0, 0}) == 0);
    assert(path_finder.uv_to_vec(point_uv{4, 4}) == EMPTY_TEST_GRID_HEIGHT * EMPTY_TEST_GRID_WIDTH - 1);
    assert(path_finder.vec_to_uv(0) == (point_uv{0, 0}));
    assert(path_finder.vec_to_uv(24) == (point_uv{4, 4}));
    std::cout << "Base Path Finder constructor tests passed" << std::endl;

    std::vector<point_uv> neighbors = path_finder.get_valid_neighbors(point_uv{0, 0}, false);
    assert(neighbors.size() == 2);
    assert(neighbors.at(0) == (point_uv{1, 0}));
    assert(neighbors.at(1) == (point_uv{0, 1}));
    std::cout << "Base Path Finder get valid neighbors tests passed" << std::endl;

    std::vector<point_uv> neighbors_diags = path_finder.get_valid_neighbors(point_uv{0, 0}, true);
    assert(neighbors_diags.size() == 3);
    assert(neighbors_diags.at(0) == (point_uv{1, 0}));
    assert(neighbors_diags.at(1) == (point_uv{0, 1}));
    assert(neighbors_diags.at(2) == (point_uv{1, 1}));
    std::cout << "Base Path Finder get valid neighbors with diags tests passed" << std::endl;

    std::vector<point_uv> neighbors_center = path_finder.get_valid_neighbors(point_uv{2, 2}, false);
    assert(neighbors_center.size() == 4);
    assert(neighbors_center.at(0) == (point_uv{3, 2}));
    assert(neighbors_center.at(1) == (point_uv{1, 2}));
    assert(neighbors_center.at(2) == (point_uv{2, 3}));
    assert(neighbors_center.at(3) == (point_uv{2, 1}));
    std::cout << "Base Path Finder get valid neighbors in center case tests passed" << std::endl;

    std::vector<point_uv> neighbors_center_diags = path_finder.get_valid_neighbors(point_uv{2, 2}, true);
    assert(neighbors_center_diags.size() == 8);
    assert(neighbors_center_diags.at(0) == (point_uv{3, 2}));
    assert(neighbors_center_diags.at(1) == (point_uv{1, 2}));
    assert(neighbors_center_diags.at(2) == (point_uv{2, 3}));
    assert(neighbors_center_diags.at(3) == (point_uv{2, 1}));
    assert(neighbors_center_diags.at(4) == (point_uv{3, 3}));
    assert(neighbors_center_diags.at(5) == (point_uv{3, 1}));
    assert(neighbors_center_diags.at(6) == (point_uv{1, 3}));
    assert(neighbors_center_diags.at(7) == (point_uv{1, 1}));
    std::cout << "Base Path Finder get valid neighbors in center case with diags tests passed" << std::endl;

    // 

    // PathFinderTestDouble path_finder_obstacles(OBSTACLE_TEST_GRID, OBSTACLE_TEST_GRID_HEIGHT, OBSTACLE_TEST_GRID_WIDTH);
    // std::vector<point_uv> neighbors = path_finder_obstacles.get_valid_neighbors(point_uv{0,0});
    // assert(neighbors.size() == 1);
    // assert(neighbors.at(0) == (point_uv{1, 0}));
    // std::cout << "BFS Path Finder constructor tests passed" << std::endl;

    return true;
}

bool test_bfs_path_finder() {
    BFS bfs(EMPTY_TEST_GRID, EMPTY_TEST_GRID_HEIGHT, EMPTY_TEST_GRID_WIDTH);
    assert(bfs.get_height() == EMPTY_TEST_GRID_HEIGHT);
    assert(bfs.get_width() == EMPTY_TEST_GRID_WIDTH);
    assert(bfs.uv_to_vec(point_uv{0, 0}) == 0);
    assert(bfs.uv_to_vec(point_uv{4, 4}) == EMPTY_TEST_GRID_HEIGHT * EMPTY_TEST_GRID_WIDTH - 1);
    assert(bfs.vec_to_uv(0) == (point_uv{0, 0}));
    assert(bfs.vec_to_uv(24) == (point_uv{4, 4}));
    std::cout << "BFS Path Finder constructor tests passed" << std::endl;

    point_uv start = {0, 0};
    point_uv end = {4, 4};
    std::optional<std::vector<point_uv>> path = bfs.find_path(start, end, false);
    assert(path);
    assert(path->size() == 9);
    assert(path->at(0) == start);
    assert(path->at(8) == end);
    std::cout << "BFS Path Finder find path no obstacles, no diags tests passed" << std::endl;

    path = bfs.find_path(start, end, true);
    assert(path);
    assert(path->size() == 5);
    assert(path->at(0) == start);
    assert(path->at(4) == end);
    std::cout << "BFS Path Finder find path no obstacles, with diags tests passed" << std::endl;

    BFS bfs_obstacles(OBSTACLE_TEST_GRID, OBSTACLE_TEST_GRID_HEIGHT, OBSTACLE_TEST_GRID_WIDTH);
    path = bfs_obstacles.find_path(start, end, false);
    assert(path);
    assert(path->size() == 15);
    assert(path->at(0) == start);
    assert(path->at(14) == end);
    std::cout << "BFS Path Finder find path with obstacles, no diags tests passed" << std::endl;

    path = bfs_obstacles.find_path(start, end, true);
    assert(path);
    assert(path->size() == 11);
    assert(path->at(0) == start);
    assert(path->at(10) == end);
    std::cout << "BFS Path Finder find path with obstacles, with diags tests passed" << std::endl;
    return true;
}

bool test_dijkstra_path_finder() {
    Dijkstra dijkstra(EMPTY_TEST_GRID, EMPTY_TEST_GRID_HEIGHT, EMPTY_TEST_GRID_WIDTH);
    assert(dijkstra.get_height() == EMPTY_TEST_GRID_HEIGHT);
    assert(dijkstra.get_width() == EMPTY_TEST_GRID_WIDTH);
    assert(dijkstra.uv_to_vec(point_uv{0, 0}) == 0);
    assert(dijkstra.uv_to_vec(point_uv{4, 4}) == EMPTY_TEST_GRID_HEIGHT * EMPTY_TEST_GRID_WIDTH - 1);
    assert(dijkstra.vec_to_uv(0) == (point_uv{0, 0}));
    assert(dijkstra.vec_to_uv(24) == (point_uv{4, 4}));
    std::cout << "Dijkstra Path Finder constructor tests passed" << std::endl;

    point_uv start = {0, 0};
    point_uv end = {4, 4};
    std::optional<std::vector<point_uv>> path = dijkstra.find_path(start, end, false);
    assert(path);
    assert(path->size() == 9);
    assert(path->at(0) == start);
    assert(path->at(8) == end);
    std::cout << "Dijkstra Path Finder find path no obstacles, no diags tests passed" << std::endl;

    path = dijkstra.find_path(start, end, true);
    assert(path);
    assert(path->size() == 5);
    assert(path->at(0) == start);
    assert(path->at(4) == end);
    std::cout << "Dijkstra Path Finder find path no obstacles, with diags tests passed" << std::endl;

    Dijkstra dijkstra_obstacles(OBSTACLE_TEST_GRID, OBSTACLE_TEST_GRID_HEIGHT, OBSTACLE_TEST_GRID_WIDTH);
    path = dijkstra_obstacles.find_path(start, end, false);
    assert(path);
    assert(path->size() == 15);
    assert(path->at(0) == start);
    assert(path->at(14) == end);
    std::cout << "Dijkstra Path Finder find path with obstacles, no diags tests passed" << std::endl;

    path = dijkstra_obstacles.find_path(start, end, true);
    assert(path);
    assert(path->size() == 11);
    assert(path->at(0) == start);
    assert(path->at(10) == end);
    std::cout << "Dijkstra Path Finder find path with obstacles, with diags tests passed" << std::endl;

    Dijkstra dijkstra_weighted(WEIGHTED_TEST_GRID, EMPTY_TEST_GRID_HEIGHT, EMPTY_TEST_GRID_WIDTH);
    path = dijkstra_weighted.find_path(start, end, false);
    assert(path);
    assert(path->size() == 15);
    assert(path->at(0) == start);
    assert(path->at(14) == end);
    std::cout << "Dijkstra Path Finder find path weighted, no diags tests passed" << std::endl;

    path = dijkstra_weighted.find_path(start, end, true);
    assert(path);
    assert(path->size() == 11);
    assert(path->at(0) == start);
    assert(path->at(10) == end);
    std::cout << "Dijkstra Path Finder find path weighted, with diags tests passed" << std::endl;

    return true;
}

bool test_astar_path_finder() { 


    AStarPathFinder astarpathfinder(EMPTY_TEST_GRID, EMPTY_TEST_GRID_HEIGHT, EMPTY_TEST_GRID_WIDTH);
    assert(astarpathfinder.get_height() == EMPTY_TEST_GRID_HEIGHT);
    assert(astarpathfinder.get_width() == EMPTY_TEST_GRID_WIDTH);
    assert(astarpathfinder.uv_to_vec(point_uv{0, 0}) == 0);
    assert(astarpathfinder.uv_to_vec(point_uv{4, 4}) == EMPTY_TEST_GRID_HEIGHT * EMPTY_TEST_GRID_WIDTH - 1);
    assert(astarpathfinder.vec_to_uv(0) == (point_uv{0, 0}));
    assert(astarpathfinder.vec_to_uv(24) == (point_uv{4, 4}));
    std::cout << "Astar Path Finder constructor tests passed" << std::endl;

    double target_euclidean_cost = sqrt(pow(4 - 0, 2) + pow(4 - 0, 2));
    assert(std::abs(astarpathfinder.heuristic_cost(point_uv{0, 0}, point_uv{4, 4}, AStarPathFinder::DistanceMetric::Euclidean) - target_euclidean_cost) < 1e-6);
    double target_manhattan_cost = std::abs(4 - 0) + std::abs(4 - 0);
    assert(std::abs(astarpathfinder.heuristic_cost(point_uv{0, 0}, point_uv{4, 4}, AStarPathFinder::DistanceMetric::Manhattan) - target_manhattan_cost) < 1e-6);
    std::cout << "Astar Path Finder heuristic cost tests passed" << std::endl;

    point_uv start = {0, 0};
    point_uv end = {4, 4};
    std::optional<std::vector<point_uv>> path = astarpathfinder.find_path(start, end, false);
    assert(path);
    assert(path->size() == 9);
    assert(path->at(0) == start);
    assert(path->at(8) == end);
    std::cout << "AStar Path Finder find path no obstacles, no diags tests passed" << std::endl;

    path = astarpathfinder.find_path(start, end, true);
    assert(path);
    assert(path->size() == 5);
    assert(path->at(0) == start);
    assert(path->at(4) == end);
    std::cout << "Astar Path Finder find path no obstacles, with diags tests passed" << std::endl;

    AStarPathFinder astarobstacles(OBSTACLE_TEST_GRID, OBSTACLE_TEST_GRID_HEIGHT, OBSTACLE_TEST_GRID_WIDTH);
    path = astarobstacles.find_path(start, end, false);
    assert(path);
    assert(path->size() == 15);
    assert(path->at(0) == start);
    assert(path->at(14) == end);
    std::cout << "Astar Path Finder find path with obstacles, no diags tests passed" << std::endl;

    path = astarobstacles.find_path(start, end, true);
    assert(path);
    assert(path->size() == 11);
    assert(path->at(0) == start);
    assert(path->at(10) == end);
    std::cout << "Astar Path Finder find path with obstacles, with diags tests passed" << std::endl;

    AStarPathFinder astar_weighted(WEIGHTED_TEST_GRID, EMPTY_TEST_GRID_HEIGHT, EMPTY_TEST_GRID_WIDTH);
    path = astar_weighted.find_path(start, end, false, 1.0, AStarPathFinder::DistanceMetric::Manhattan);
    assert(path);
    std::cout << "Length of path: " << path->size() << std::endl;
    for (size_t i = 0; i < path->size(); ++i) {
        std::cout << "Path point " << i << ": (" << path->at(i).u << ", " << path->at(i).v << ")" << std::endl;
    }
    assert(path->size() == 15);
    assert(path->at(0) == start);
    assert(path->at(14) == end);
    std::cout << "Astar Path Finder find path weighted, no diags tests passed" << std::endl;

    path = astar_weighted.find_path(start, end, true, 1.0, AStarPathFinder::DistanceMetric::Euclidean);
    assert(path);
    assert(path->size() == 11);
    assert(path->at(0) == start);
    assert(path->at(10) == end);
    std::cout << "Astar Path Finder find path weighted, with diags tests passed" << std::endl;

    // with this heavy heuristic weight, we should get get the shortest possisble path with diags
    path = astar_weighted.find_path(start, end, true, 5.0, AStarPathFinder::DistanceMetric::Euclidean);
    assert(path);
    assert(path->size() == 11);
    assert(path->at(0) == start);
    assert(path->at(10) == end);
    std::cout << "Astar Path Finder find path weighted, with diags, heuristic weight 5 tests passed" << std::endl;

    return true;


 };

int main() {
    test_base_path_finder();
    test_bfs_path_finder();
    test_dijkstra_path_finder();
    test_astar_path_finder();
    return 0;
}