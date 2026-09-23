#include "axis_grasp/adapters/polygon_mask.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace axis_grasp {
namespace {

bool PointOnSegment(double px, double py, const Vec2d& a, const Vec2d& b) {
  const double cross = (px - a.x) * (b.y - a.y) -
                       (py - a.y) * (b.x - a.x);
  if (std::abs(cross) > 1e-9) return false;
  return px >= std::min(a.x, b.x) && px <= std::max(a.x, b.x) &&
         py >= std::min(a.y, b.y) && py <= std::max(a.y, b.y);
}

// Fills only the polygon's integer bounding box. Equivalent to scanning every
// pixel: PointInPolygon can only be true inside or on an edge of the polygon,
// and both lie within the box spanned by its vertices.
void FillPolygon(const std::vector<Vec2d>& polygon, Image<std::uint8_t>* mask) {
  if (polygon.size() < 3) return;
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const Vec2d& point : polygon) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  const int x0 = std::max(0, static_cast<int>(std::floor(min_x)));
  const int y0 = std::max(0, static_cast<int>(std::floor(min_y)));
  const int x1 = std::min(mask->width() - 1, static_cast<int>(std::ceil(max_x)));
  const int y1 = std::min(mask->height() - 1, static_cast<int>(std::ceil(max_y)));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (PointInPolygon(x, y, polygon)) (*mask)(y, x) = 1;
    }
  }
}

}  // namespace

bool PointInPolygon(double x, double y, const std::vector<Vec2d>& polygon) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (PointOnSegment(x, y, polygon[j], polygon[i])) return true;
    const bool crosses = ((polygon[i].y > y) != (polygon[j].y > y)) &&
                         (x < (polygon[j].x - polygon[i].x) *
                                      (y - polygon[i].y) /
                                      (polygon[j].y - polygon[i].y) +
                                  polygon[i].x);
    if (crosses) inside = !inside;
  }
  return inside;
}

Image<std::uint8_t> RasterizePolygons(
    const std::vector<std::vector<Vec2d>>& polygons, int width, int height) {
  Image<std::uint8_t> mask(height, width, 0);
  for (const std::vector<Vec2d>& polygon : polygons) {
    FillPolygon(polygon, &mask);
  }
  return mask;
}

std::vector<std::vector<Vec2d>> ParseFlatContour(
    const std::vector<std::uint16_t>& flat) {
  std::vector<std::vector<Vec2d>> polygons;
  std::vector<Vec2d> current;
  const std::size_t points = flat.size() / 2;
  for (std::size_t i = 0; i < points; ++i) {
    const double x = flat[i * 2];
    const double y = flat[i * 2 + 1];
    if (x == 0.0 && y == 0.0) {
      if (current.size() >= 3) polygons.push_back(current);
      current.clear();
      continue;
    }
    current.push_back({x, y});
  }
  if (current.size() >= 3) polygons.push_back(current);
  return polygons;
}

void ScalePolygon(std::vector<Vec2d>* polygon, int source_width,
                  int source_height, int destination_width,
                  int destination_height) {
  if (polygon == nullptr) return;
  if (source_width <= 0 || source_height <= 0 || destination_width <= 0 ||
      destination_height <= 0) {
    return;
  }
  const double sx = static_cast<double>(destination_width) / source_width;
  const double sy = static_cast<double>(destination_height) / source_height;
  for (Vec2d& point : *polygon) {
    point.x *= sx;
    point.y *= sy;
  }
}

Result<Image<std::uint8_t>> RasterizeFlatContours(
    const std::vector<std::vector<std::uint16_t>>& contours, int source_width,
    int source_height, int destination_width, int destination_height) {
  if (source_width <= 0 || source_height <= 0) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "detection image size must be positive");
  }
  if (destination_width <= 0 || destination_height <= 0) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "destination image size must be positive");
  }
  std::vector<std::vector<Vec2d>> polygons;
  for (const std::vector<std::uint16_t>& flat : contours) {
    for (std::vector<Vec2d>& polygon : ParseFlatContour(flat)) {
      ScalePolygon(&polygon, source_width, source_height, destination_width,
                   destination_height);
      polygons.push_back(std::move(polygon));
    }
  }
  if (polygons.empty()) {
    return Status::Error(
        ErrorCode::kEmptyRoi,
        "no detection contour held a polygon with three or more vertices");
  }
  return RasterizePolygons(polygons, destination_width, destination_height);
}

}  // namespace axis_grasp
