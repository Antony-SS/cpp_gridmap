#pragma once

#include <cmath>
#include <optional>
#include <vector>

/**
 * Axis-aligned rectangular bounds in continuous (x, y) space.
 *
 * Attributes
 * ----------
 * y_bound_top : double
 *     Maximum y coordinate (forward in FLU).
 * y_bound_bottom : double
 *     Minimum y coordinate.
 * x_bound_left : double
 *     Minimum x coordinate (left in FLU).
 * x_bound_right : double
 *     Maximum x coordinate.
 */
struct gridmap_bounds {
    double y_bound_upper;
    double y_bound_lower;
    double x_bound_upper;
    double x_bound_lower;
};

/**
 * A point in continuous (x, y) space.
 *
 * Attributes
 * ----------
 * x : double
 *     Horizontal coordinate.
 * y : double
 *     Vertical coordinate.
 */
struct point_xy {
    double const x;
    double const y;
};

/**
 * A point in discrete grid (u, v) space.
 *
 * Attributes
 * ----------
 * u : int
 *     Row index.
 * v : int
 *     Column index.
 */
struct point_uv {
    int const u;
    int const v;
};

/**
 * Bidirectional mapping between continuous (x, y) coordinates and
 * discrete grid (u, v) indices, constructed in the FLU frame.
 *
 * Attributes
 * ----------
 * bounds : gridmap_bounds
 *     Axis-aligned spatial extent of the grid.
 * resolution : double
 *     Side length of each grid cell in metres.
 *
 * Notes
 * -----
 * The right / bottom bounds may be extended slightly so that the
 * span is evenly divisible by resolution.
 */
class GridMapCoordinates {
private:
    gridmap_bounds bounds;
    double const resolution;

public:
    GridMapCoordinates(gridmap_bounds bounds, double resolution);

    /** Return the grid's spatial bounds. */
    gridmap_bounds get_bounds() const { return bounds; }

    /** Return the grid height in metres. */
    double get_height_m() const { return bounds.x_bound_upper - bounds.x_bound_lower; }

    /** Return the grid width in metres. */
    double get_width_m() const { return bounds.y_bound_upper - bounds.y_bound_lower; }

    /** Return the cell resolution in metres. */
    double get_resolution() const { return resolution; }

    /** Return the grid width in cells. */
    int get_width_cells() const { return std::ceil(get_width_m() / resolution); }

    /** Return the grid height in cells. */
    int get_height_cells() const { return std::ceil(get_height_m() / resolution); }

    /**
     * Convert a continuous (x, y) point to grid (u, v) indices.
     *
     * Parameters
     * ----------
     * point : point_xy
     *     Continuous coordinate to convert.
     *
     * Returns
     * -------
     * std::optional<point_uv>
     *     Corresponding grid indices, or std::nullopt if out of bounds.
     */
    std::optional<point_uv> xy_to_vu(point_xy const& point) const;

    /**
     * Convert a grid (u, v) index pair to continuous (x, y).
     *
     * Parameters
     * ----------
     * point : point_uv
     *     Grid indices to convert.
     *
     * Returns
     * -------
     * point_xy
     *     Corresponding continuous coordinates.
     */
    point_xy vu_to_xy(point_uv const& point) const;

    /**
     * Batch-convert continuous (x, y) points to grid (u, v) indices.
     *
     * Parameters
     * ----------
     * points : std::vector<point_xy>
     *     Continuous coordinates to convert.
     *
     * Returns
     * -------
     * std::vector<point_uv>
     *     Corresponding grid row and column indices.
     */
    std::vector<point_uv> xy_to_vu(std::vector<point_xy> const& points) const;

    /**
     * Batch-convert grid (u, v) index pairs to continuous (x, y).
     *
     * Parameters
     * ----------
     * points : std::vector<point_uv>
     *     Grid indices to convert.
     *
     * Returns
     * -------
     * std::vector<point_xy>
     *     Corresponding continuous coordinates.
     */
    std::vector<point_xy> vu_to_xy(std::vector<point_uv> const& points) const;
};

/**
 * A dense integer grid backed by a flat vector.
 *
 * Each cell stores an integer value (e.g. occupancy count) and is
 * addressed by grid (u, v) indices via the embedded coordinate system.
 *
 * Attributes
 * ----------
 * gridmap_coords : GridMapCoordinates
 *     Coordinate system shared with callers for point conversion.
 * data : std::vector<int>
 *     Row-major flat storage, sized width_cells * height_cells.
 */
class DenseGridLayer {

    public:
        /**
         * Construct a DenseGridLayer with a provided coordinate system.
         *
         * Parameters
         * ----------
         * gridmap_coords : GridMapCoordinates
         *     Coordinate system shared with callers for point conversion.
         *
         * Notes
         * -----
         * The grid data will be initialized to all zeros based on the gridmap's
         * dimensions.
         */
        DenseGridLayer(GridMapCoordinates gridmap_coords);

        /**
         * Construct a DenseGridLayer with a provided coordinate system and data.
         *
         * Parameters
         * ----------
         * gridmap_coords : GridMapCoordinates
         *     Coordinate system shared with callers for point conversion.
         * data : std::vector<int>
         *     Flat, row-major storage of integer values. Must match grid size.
         */
        DenseGridLayer(GridMapCoordinates gridmap_coords, std::vector<int> data);

        GridMapCoordinates gridmap_coords;

        /**
         * Increment a cell's value by a given weight.
         *
         * Parameters
         * ----------
         * point : point_uv
         *     Grid cell to update.
         * weight : int, optional
         *     Value to add (default 1).
         */
        void increment_point(point_uv const& point, int weight = 1);

        /**
         * Return the value stored at a grid cell.
         *
         * Parameters
         * ----------
         * point : point_uv
         *     Grid cell to query.
         *
         * Returns
         * -------
         * int
         *     Current cell value.
         */
        int get_point(point_uv const& point);

        /** Return a read-only reference to the underlying flat data. */
        std::vector<int> const& get_data() const { return data; }

        /** Reset all cell values to zero. */
        void clear_grid();

    private:
        std::vector<int> data;

}

// class GridMap {

//     public:



//     private:




//     protected:


// }
