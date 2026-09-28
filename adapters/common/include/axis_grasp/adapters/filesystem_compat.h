#ifndef AXIS_GRASP_ADAPTERS_FILESYSTEM_COMPAT_H_
#define AXIS_GRASP_ADAPTERS_FILESYSTEM_COMPAT_H_

// std::filesystem is C++17, but libstdc++ only ships the <filesystem> header
// from GCC 8 on. GCC 7 has the same feature under its pre-standard name in
// <experimental/filesystem>, and no <filesystem> header to include at all —
// which fails as "fatal error: filesystem: No such file or directory". The
// vehicle builds with GCC 7.5 (Ubuntu 18.04 / ROS Melodic), so include this
// header instead of <filesystem> and spell paths axis_grasp::fs::.
//
// GCC 7 and 8 also keep the implementation in a separate libstdc++fs archive;
// adapters/common/CMakeLists.txt links it when the compiler is older than 9.
//
// The alias alone is not the whole port. GCC 7 implements the filesystem TS,
// which predates the C++17 additions to directory_entry: query the path with the
// free functions (fs::is_directory(entry.path())) rather than the entry itself
// (entry.is_directory()).

#if defined(__has_include)
#if __has_include(<filesystem>) && __cplusplus >= 201703L
#define AXIS_GRASP_HAS_STD_FILESYSTEM 1
#endif
#endif

#ifdef AXIS_GRASP_HAS_STD_FILESYSTEM
#include <filesystem>
namespace axis_grasp {
namespace fs = std::filesystem;
}  // namespace axis_grasp
#else
#include <experimental/filesystem>
namespace axis_grasp {
namespace fs = std::experimental::filesystem;
}  // namespace axis_grasp
#endif

#endif  // AXIS_GRASP_ADAPTERS_FILESYSTEM_COMPAT_H_
