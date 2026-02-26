#pragma once




/** Coordinate types for continuous (x, y) and discrete (u, v) space. */
struct point2 {
    double const x;
    double const y;
};

struct point3 {
    double const x;
    double const y;
    double const z;
};

/** A point in discrete (u, v) space. */
struct point_uv {
    int const u;
    int const v;
};