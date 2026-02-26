#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <data_models.h>
/**
 * Axis-aligned rectangular bounds in continuous (x, y) space.
 *
 * Attributes
 * ----------
 * y_bound_upper : double
 *     Maximum y coordinate (forward in FLU).
 * y_bound_lower : double
 *     Minimum y coordinate.
 * x_bound_upper : double
 *     Maximum x coordinate.
 * x_bound_lower : double
 *     Minimum x coordinate.
 */
struct gridmap_bounds {
    double y_bound_upper;
    double y_bound_lower;
    double x_bound_upper;
    double x_bound_lower;
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
        // std::vector<double>x_edges;
        // std::vector<double>y_edges;
        // std::vector<double>x_bin_centers; consider adding these
        // std::vector<double>y_bin_centers;

    public:

        std::string const map_frame = "FLU";  // only supporting FLU for now, but extension to ENU is trivial

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
         * point : point2
         *     Continuous coordinate to convert.
         *
         * Returns
         * -------
         * std::optional<point_uv>
         *     Corresponding grid indices, or std::nullopt if out of bounds.
         */
        std::optional<point_uv> xy_to_uv(point2 const& point) const;

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
         * point2
         *     Corresponding continuous coordinates.
         */
        point2 uv_to_xy(point_uv const& point) const;

        /**
         * Batch-convert continuous (x, y) points to grid (u, v) indices.
         *
         * Parameters
         * ----------
         * points : std::vector<point2>
         *     Continuous coordinates to convert.
         *
         * Returns
         * -------
         * std::vector<point_uv>
         *     Corresponding grid row and column indices.
         */
        std::vector<point_uv> xy_to_uv(std::vector<point2> const& points) const;

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
         * std::vector<point2>
         *     Corresponding continuous coordinates.
         */
        std::vector<point2> uv_to_xy(std::vector<point_uv> const& points) const;
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

        GridMapCoordinates gridmap_coords;
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

        /**
         * Convert grid (u, v) indices to flat vector index.
         *
         * Parameters
         * ----------
         * point : point_uv
         *     Grid cell indices (u, v).
         *
         * Returns
         * -------
         * int
         *     Index into the flat data vector corresponding to the cell.
         */
        int uv_to_vec_idx(point_uv const& point) const { 
            return point.v * this->gridmap_coords.get_width_cells() + point.u; 
        }

        /**
         * Convert flat vector index to grid (u, v) indices.
         *
         * Parameters
         * ----------
         * idx : int
         *     Index into the flat data vector.
         *
         * Returns
         * -------
         * point_uv
         *     Grid cell indices (u, v) corresponding to the flat vector index.
         *
         * Notes
         * -----
         * Row-major ordering: u = idx % width, v = idx / width.
         */
        point_uv vec_idx_to_uv(int idx) const { 
            return point_uv{
                idx % this->gridmap_coords.get_width_cells(), 
                idx / this->gridmap_coords.get_width_cells()
            }; 
        }

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
        int get_point(point_uv const& point) const;

        /** Return a read-only reference to the underlying flat data. */
        std::vector<int> const& get_data() const { return data; }

        /** Visaulize the grid as a heatmap */
        void visualize_grid(std::string const& output_path, bool binary_grid = false) const;

        /** Reset all cell values to zero. */
        void clear_grid() { this->data = std::vector<int>(this->data.size(), 0); }

    private:
        std::vector<int> data; // stores in row major form (if you appended all rows to eachother, starting w/ first one)

        /** Utility function for writing a pgm */
        void write_pgm(std::string const& filepath, std::vector<int> data, int width, int height, int max_value = 255) const;

};
