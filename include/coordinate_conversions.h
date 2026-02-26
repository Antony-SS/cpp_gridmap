#pragma once

#include <vector>
#include <data_models.h>


constexpr point2 flu_to_enu(point2 const& point) {
    return point2{- point.y, point.x};
}

constexpr point2 enu_to_flu(point2 const& point) {
    return point2{point.y, -point.x};
}

std::vector<point2> flu_to_enu(std::vector<point2> const& points) {
    std::vector<point2> converted_points;
    for (const auto& point : points) {
        converted_points.push_back(flu_to_enu(point));
    }
    return converted_points;
}

std::vector<point2> enu_to_flu(std::vector<point2> const& points) {
    std::vector<point2> converted_points;
    for (const auto& point : points) {
        converted_points.push_back(enu_to_flu(point));
    }
    return converted_points;
}



