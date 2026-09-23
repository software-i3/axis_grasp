#ifndef AXIS_GRASP_ADAPTERS_POLYGON_MASK_H_
#define AXIS_GRASP_ADAPTERS_POLYGON_MASK_H_

#include <cstdint>
#include <vector>

#include "axis_grasp/core/image.h"
#include "axis_grasp/core/math_types.h"
#include "axis_grasp/core/status.h"

namespace axis_grasp {

// Contour and polygon rasterisation shared by every adapter that has to turn a
// vector outline into the binary ROI mask Pipeline consumes. ROS-free so the
// dataset and ROS paths can be tested against the same code.

// Ray-casting point-in-polygon test. Points exactly on an edge count as inside.
// Returns false for a polygon with fewer than three vertices.
bool PointInPolygon(double x, double y, const std::vector<Vec2d>& polygon);

// Union-fill polygons into a width x height mask: 1 inside any polygon, else 0.
// Polygons with fewer than three vertices are ignored.
Image<std::uint8_t> RasterizePolygons(
    const std::vector<std::vector<Vec2d>>& polygons, int width, int height);

// Split a flat [x1 y1 x2 y2 ...] contour on its (0, 0) separators, as
// DetectedInstance.msg defines. Each (0, 0) closes the current polygon and
// starts a new one; runs with fewer than three vertices are dropped.
std::vector<std::vector<Vec2d>> ParseFlatContour(
    const std::vector<std::uint16_t>& flat);

// Scale a polygon from the source image frame onto the destination frame.
// No-op unless both frames have positive dimensions.
void ScalePolygon(std::vector<Vec2d>* polygon, int source_width,
                  int source_height, int destination_width,
                  int destination_height);

// Scale and union-fill flat detector contours into one destination-frame mask.
// Contours are in the detector's own image frame, which generally differs from
// the destination resolution, so every contour is scaled by the ratio of the
// two frames. Returns kEmptyRoi when no contour held three or more vertices.
Result<Image<std::uint8_t>> RasterizeFlatContours(
    const std::vector<std::vector<std::uint16_t>>& contours, int source_width,
    int source_height, int destination_width, int destination_height);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_ADAPTERS_POLYGON_MASK_H_
