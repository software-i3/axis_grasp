#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "axis_grasp/adapters/polygon_mask.h"
#include "axis_grasp/core/image.h"
#include "axis_grasp/core/math_types.h"
#include "axis_grasp/core/status.h"

namespace axis_grasp {
namespace {

std::vector<std::uint16_t> FlatContour(
    const std::vector<std::pair<int, int>>& points) {
  std::vector<std::uint16_t> flat;
  for (const std::pair<int, int>& point : points) {
    flat.push_back(static_cast<std::uint16_t>(point.first));
    flat.push_back(static_cast<std::uint16_t>(point.second));
  }
  return flat;
}

// The real capture that motivated the clip signal in mine_centering: the sim's
// detector reported a limpet contour whose last vertices sit on row 0 of its
// 128x72 frame. Reused here because it exercises a contour that touches a frame
// edge and is therefore clipped.
const std::vector<std::pair<int, int>> kLimpetFrontCam = {
    {47, 0}, {84, 0}, {88, 14}, {80, 24}, {52, 26}, {45, 13}};

TEST(ParseFlatContourTest, SplitsOnZeroZeroSeparators) {
  std::vector<std::uint16_t> flat = FlatContour({{1, 1}, {5, 1}, {5, 5}});
  flat.push_back(0);
  flat.push_back(0);
  const std::vector<std::uint16_t> second = FlatContour({{2, 2}, {6, 2}, {6, 6}});
  flat.insert(flat.end(), second.begin(), second.end());

  const std::vector<std::vector<Vec2d>> polygons = ParseFlatContour(flat);
  ASSERT_EQ(polygons.size(), 2U);
  EXPECT_EQ(polygons[0].size(), 3U);
  EXPECT_DOUBLE_EQ(polygons[0][0].x, 1.0);
  EXPECT_DOUBLE_EQ(polygons[0][0].y, 1.0);
  EXPECT_EQ(polygons[1].size(), 3U);
  EXPECT_DOUBLE_EQ(polygons[1][0].x, 2.0);
  EXPECT_DOUBLE_EQ(polygons[1][2].y, 6.0);
}

TEST(ParseFlatContourTest, DropsRunsWithFewerThanThreeVertices) {
  // A two-vertex run before a separator and a single vertex after it.
  std::vector<std::uint16_t> flat = FlatContour({{1, 1}, {5, 1}});
  flat.push_back(0);
  flat.push_back(0);
  flat.insert(flat.end(), {7, 7});

  EXPECT_TRUE(ParseFlatContour(flat).empty());
}

TEST(ParseFlatContourTest, KeepsATrailingPolygonWithNoSeparator) {
  const std::vector<std::uint16_t> flat =
      FlatContour({{1, 1}, {5, 1}, {5, 5}, {1, 5}});
  const std::vector<std::vector<Vec2d>> polygons = ParseFlatContour(flat);
  ASSERT_EQ(polygons.size(), 1U);
  EXPECT_EQ(polygons[0].size(), 4U);
}

TEST(ParseFlatContourTest, IgnoresAnOddTrailingValue) {
  std::vector<std::uint16_t> flat = FlatContour({{1, 1}, {5, 1}, {5, 5}});
  flat.push_back(42);
  const std::vector<std::vector<Vec2d>> polygons = ParseFlatContour(flat);
  ASSERT_EQ(polygons.size(), 1U);
  EXPECT_EQ(polygons[0].size(), 3U);
}

TEST(ScalePolygonTest, AppliesTheDetectorToDisparityRatio) {
  // The front camera reports 128x72 against a 480x270 depth image: 3.75x on
  // both axes.
  std::vector<Vec2d> polygon = {{0.0, 0.0}, {128.0, 72.0}};
  ScalePolygon(&polygon, 128, 72, 480, 270);
  EXPECT_DOUBLE_EQ(polygon[0].x, 0.0);
  EXPECT_DOUBLE_EQ(polygon[0].y, 0.0);
  EXPECT_DOUBLE_EQ(polygon[1].x, 480.0);
  EXPECT_DOUBLE_EQ(polygon[1].y, 270.0);
}

TEST(ScalePolygonTest, LeavesThePolygonAloneForDegenerateFrames) {
  std::vector<Vec2d> polygon = {{3.0, 4.0}};
  ScalePolygon(&polygon, 0, 72, 480, 270);
  EXPECT_DOUBLE_EQ(polygon[0].x, 3.0);
  EXPECT_DOUBLE_EQ(polygon[0].y, 4.0);
}

TEST(RasterizePolygonsTest, MatchesAFullImageScan) {
  // RasterizePolygons fills only the polygon's bounding box. That is an
  // optimisation, so it has to agree with the exhaustive scan it replaced on
  // every pixel, including the ones it skips.
  const std::vector<Vec2d> polygon = {{2.0, 3.0},  {11.0, 1.0}, {17.0, 9.0},
                                      {9.0, 14.0}, {1.0, 10.0}};
  const int width = 24;
  const int height = 18;
  const Image<std::uint8_t> fast =
      RasterizePolygons({polygon}, width, height);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::uint8_t expected =
          PointInPolygon(x, y, polygon) ? std::uint8_t{1} : std::uint8_t{0};
      ASSERT_EQ(fast(y, x), expected) << "at (" << x << ", " << y << ")";
    }
  }
}

TEST(RasterizePolygonsTest, UnitesOverlappingPolygons) {
  // Both rectangles span y 1..4 in a 8x6 frame, so there is clear background
  // above, below, and to the left to check against. Note that a point on an
  // edge counts as inside, so the test avoids exact edge coordinates.
  const std::vector<Vec2d> left = {{1.0, 1.0}, {4.0, 1.0}, {4.0, 4.0}, {1.0, 4.0}};
  const std::vector<Vec2d> right = {{3.0, 1.0}, {6.0, 1.0}, {6.0, 4.0}, {3.0, 4.0}};
  const Image<std::uint8_t> mask = RasterizePolygons({left, right}, 8, 6);
  EXPECT_EQ(mask(2, 2), 1);  // Interior of the left rectangle.
  EXPECT_EQ(mask(2, 5), 1);  // Interior of the right rectangle.
  EXPECT_EQ(mask(2, 4), 1);  // The overlap, filled by both.
  EXPECT_EQ(mask(0, 0), 0);  // Above and left of both.
  EXPECT_EQ(mask(5, 3), 0);  // Below both.
}

TEST(RasterizePolygonsTest, RasterizesNothingForADegeneratePolygon) {
  const std::vector<Vec2d> line = {{0.0, 0.0}, {5.0, 5.0}};
  const Image<std::uint8_t> mask = RasterizePolygons({line}, 8, 8);
  for (std::size_t i = 0; i < mask.size(); ++i) EXPECT_EQ(mask[i], 0);
}

TEST(RasterizeFlatContoursTest, ScalesTheRealLimpetContourOntoTheDepthFrame) {
  const std::vector<std::uint16_t> contour = FlatContour(kLimpetFrontCam);
  Result<Image<std::uint8_t>> mask =
      RasterizeFlatContours({contour}, 128, 72, 480, 270);
  ASSERT_TRUE(mask.ok()) << mask.status().message;
  const Image<std::uint8_t>& raster = mask.value();
  ASSERT_EQ(raster.width(), 480);
  ASSERT_EQ(raster.height(), 270);

  // The contour spans x 45..88 and y 0..26 in the detector frame, so scaled by
  // 3.75 it spans x 168..330 and y 0..97 in the depth frame. The interior is
  // filled and well outside is not.
  EXPECT_EQ(raster(50, 250), 1);
  EXPECT_EQ(raster(10, 250), 1);   // Vertex row 0 is the clipped top edge.
  EXPECT_EQ(raster(5, 10), 0);
  EXPECT_EQ(raster(260, 470), 0);

  // Nothing is painted outside the scaled contour's extent.
  int max_x = -1;
  int max_y = -1;
  for (int y = 0; y < raster.height(); ++y) {
    for (int x = 0; x < raster.width(); ++x) {
      if (raster(y, x) == 0) continue;
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
  }
  EXPECT_LE(max_x, 331);
  EXPECT_LE(max_y, 98);
}

TEST(RasterizeFlatContoursTest, ReportsEmptyRoiForAContourWithNoPolygon) {
  // Two vertices: ParseFlatContour drops the run, so there is nothing to fill.
  const std::vector<std::uint16_t> contour = FlatContour({{1, 1}, {5, 1}});
  Result<Image<std::uint8_t>> mask =
      RasterizeFlatContours({contour}, 128, 72, 480, 270);
  ASSERT_FALSE(mask.ok());
  EXPECT_EQ(mask.status().code, ErrorCode::kEmptyRoi);
}

TEST(RasterizeFlatContoursTest, ReportsEmptyRoiForNoContoursAtAll) {
  const std::vector<std::vector<std::uint16_t>> none;
  Result<Image<std::uint8_t>> mask = RasterizeFlatContours(none, 128, 72, 480, 270);
  ASSERT_FALSE(mask.ok());
  EXPECT_EQ(mask.status().code, ErrorCode::kEmptyRoi);
}

TEST(RasterizeFlatContoursTest, RejectsNonPositiveFrames) {
  const std::vector<std::uint16_t> contour = FlatContour(kLimpetFrontCam);
  EXPECT_EQ(RasterizeFlatContours({contour}, 0, 72, 480, 270).status().code,
            ErrorCode::kInvalidArgument);
  EXPECT_EQ(RasterizeFlatContours({contour}, 128, 72, 480, 0).status().code,
            ErrorCode::kInvalidArgument);
}

}  // namespace
}  // namespace axis_grasp
