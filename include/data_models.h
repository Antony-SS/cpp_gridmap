#pragma once

/** Coordinate types for continuous (x, y) and discrete (u, v) space. */
struct point2 {
    double const x;
    double const y;

    bool operator==(const point2& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const point2& other) const {
        return !(*this == other);
    }
};

struct point3 {
    double const x;
    double const y;
    double const z;

    bool operator==(const point3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const point3& other) const {
        return !(*this == other);
    }
};

/** A point in discrete (u, v) space. */
struct point_uv {
    int const u;
    int const v;

    bool operator==(const point_uv& other) const {
        return u == other.u && v == other.v;
    }

    bool operator!=(const point_uv& other) const {
        return !(*this == other);
    }
};