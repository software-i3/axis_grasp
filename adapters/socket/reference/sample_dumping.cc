// sample_dumping.cc — RTP stereo stream receiver -> local file dump.
//
// Based on sample_original.cc's RTP client (uvgRTP + httplib) combined with the
// self-contained file writers from main.cc.  No fp16.h / happly.h / nlohmann.json
// dependency — all image, depth, and PLY writing is done with hand-rolled helpers.
//
// Writes per captured frame to <output_dir>/:
//   camera_info.json         (written once)
//   000042_left.ppm          (binary P6 RGB)
//   000042_right.ppm         (binary P6 RGB — only if present in RTP payload)
//   000042_depth.npy         (lossless float32, shape (H, W))
//   000042_depth_preview.pgm (8-bit normalized, for eyeballing)
//   000042.ply               (colored point cloud)
//
// The RTP payload layout expected by the standard RTPSender (DEPTH_MODE 0 / SGM):
//   NTP header (2 x int) + left RGB (W*H*3) + disparity (W*H*2, FP16)
// If the server is configured to also send the right image, it is extracted from
// extra payload bytes automatically.

#include <httplib.h>
#include <uvgrtp/lib.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Configuration (can be overridden via command-line flags)
// ---------------------------------------------------------------------------

constexpr uint16_t HTTP_PORT = 8000;       // hardcoded — do not change
constexpr uint16_t RTP_PORT  = 5600;       // default RTP port

constexpr int NTP_HEADER_SIZE = sizeof(int) * 2;

// Server-side calibration resolution (fixed by the RTPSender convention).
constexpr int CALIB_W = 1600;
constexpr int CALIB_H = 1200;

// ---------------------------------------------------------------------------
// Runtime globals (populated from CLI / server response)
// ---------------------------------------------------------------------------

static std::atomic<bool> should_stay_connected{true};
static std::atomic<bool> should_poll{true};

static std::string g_svc_address;
static std::string g_left_bus_id;
static std::string g_right_bus_id;
static std::string g_calib_file;
static std::string g_dwvo_file;
static std::string g_input_type;   // "USB" or "DWVO"

// Intrinsics received from the server (at CALIB_W x CALIB_H).
static float g_fx_calib = 0.f;
static float g_fy_calib = 0.f;     // may be zero — not always sent
static float g_cx_calib = 0.f;
static float g_cy_calib = 0.f;

static int    g_depth_mode  = 0;   // 0=SGM, 1=ML refined_fast, 2=ML refined_pro
static int    g_img_w       = 800;
static int    g_img_h       = 600;

// File-dump configuration (set from CLI flags).
static std::string g_output_dir          = "captures";
static int         g_save_every          = 1;
static int         g_num_frames          = 0;    // 0 = run until Ctrl+C
static bool        g_depth_is_disparity  = true;
static double      g_baseline_m          = 0.1;
static double      g_depth_scale         = 100.0;  // m→cm
static double      g_min_depth           = 0.0;
static double      g_max_depth           = 1000.0;
static bool        g_write_ply           = true;
static bool        g_write_images        = true;
static bool        g_write_depth_npy     = true;
static bool        g_write_depth_preview = true;

// ---------------------------------------------------------------------------
// Helpers — types
// ---------------------------------------------------------------------------

struct FrameSize { int width; int height; };

struct Intrinsics { double fx; double fy; double cx; double cy; };

// ---------------------------------------------------------------------------
// IEEE half -> float (replaces fp16.h / fp16_ieee_to_fp32_value)
// ---------------------------------------------------------------------------

inline float Fp16ToFp32(uint16_t h) {
  const uint32_t sign = static_cast<uint32_t>(h & 0x8000u) << 16;
  const uint32_t exp  = (h & 0x7C00u) >> 10;
  const uint32_t mant = h & 0x03FFu;
  uint32_t bits;
  if (exp == 0) {
    if (mant == 0) {
      bits = sign;
    } else {
      int e = 0;
      uint32_t m = mant;
      while ((m & 0x0400u) == 0) { m <<= 1; ++e; }
      m &= 0x03FFu;
      bits = sign | (static_cast<uint32_t>(127 - 15 - e) << 23) | (m << 13);
    }
  } else if (exp == 0x1F) {
    bits = sign | 0x7F800000u | (mant << 13);
  } else {
    bits = sign | ((exp + (127 - 15)) << 23) | (mant << 13);
  }
  float out;
  std::memcpy(&out, &bits, sizeof(out));
  return out;
}

// ---------------------------------------------------------------------------
// Decode depth buffer: FP16 (2 B/px) or FP32 (4 B/px) → vector<float>
// ---------------------------------------------------------------------------

static std::vector<float> DecodeDepth(const uint8_t *data, size_t bytes,
                                      int w, int h) {
  const size_t n = static_cast<size_t>(w) * h;
  if (bytes == n * 2) {
    const auto *p = reinterpret_cast<const uint16_t *>(data);
    std::vector<float> out(n);
    for (size_t i = 0; i < n; ++i) out[i] = Fp16ToFp32(p[i]);
    return out;
  }
  if (bytes == n * 4) {
    const auto *p = reinterpret_cast<const float *>(data);
    return std::vector<float>(p, p + n);
  }
  return {};
}

// ---------------------------------------------------------------------------
// Scale intrinsics from calibration resolution to working resolution.
// fx & cx by width ratio, fy & cy by height ratio (matches main.cc / RTP sample).
// ---------------------------------------------------------------------------

static Intrinsics ScaleIntrinsics(const Intrinsics &in,
                                  FrameSize from, FrameSize to) {
  const double sx = static_cast<double>(to.width)  / from.width;
  const double sy = static_cast<double>(to.height) / from.height;
  return {in.fx * sx, in.fy * sy, in.cx * sx, in.cy * sy};
}

// ---------------------------------------------------------------------------
// Binary P6 PPM writer (handles RGB, RGBA→RGB, mono→grey RGB)
// ---------------------------------------------------------------------------

static bool WritePpmRgb(const std::string &path,
                        const uint8_t *data, int w, int h, int channels) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << "P6\n" << w << " " << h << "\n255\n";
  if (channels == 3) {
    f.write(reinterpret_cast<const char *>(data),
            static_cast<std::streamsize>(static_cast<size_t>(w) * h * 3));
  } else {
    std::vector<uint8_t> buf;
    buf.reserve(static_cast<size_t>(w) * h * 3);
    const size_t n = static_cast<size_t>(w) * h;
    for (size_t i = 0; i < n; ++i) {
      if (channels == 4) {
        buf.push_back(data[i * 4 + 0]);
        buf.push_back(data[i * 4 + 1]);
        buf.push_back(data[i * 4 + 2]);
      } else {  // mono
        buf.push_back(data[i]);
        buf.push_back(data[i]);
        buf.push_back(data[i]);
      }
    }
    f.write(reinterpret_cast<const char *>(buf.data()),
            static_cast<std::streamsize>(buf.size()));
  }
  return f.good();
}

// ---------------------------------------------------------------------------
// Binary P5 PGM writer
// ---------------------------------------------------------------------------

static bool WritePgm(const std::string &path,
                     const std::vector<uint8_t> &gray, int w, int h) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << "P5\n" << w << " " << h << "\n255\n";
  f.write(reinterpret_cast<const char *>(gray.data()),
          static_cast<std::streamsize>(gray.size()));
  return f.good();
}

// ---------------------------------------------------------------------------
// Minimal little-endian .npy writer (NumPy v1.0, C-order float32)
// ---------------------------------------------------------------------------

static bool WriteNpyFloat32(const std::string &path,
                            const std::vector<float> &data, int h, int w) {
  std::ostringstream hs;
  hs << "{'descr': '<f4', 'fortran_order': False, 'shape': ("
     << h << ", " << w << "), }";
  std::string header = hs.str();
  const size_t base = 10 + header.size() + 1;
  header.append((64 - (base % 64)) % 64, ' ');
  header.push_back('\n');
  const auto hlen = static_cast<uint16_t>(header.size());
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f.write("\x93NUMPY", 6);
  const char ver[2] = {1, 0};
  f.write(ver, 2);
  f.write(reinterpret_cast<const char *>(&hlen), 2);
  f.write(header.data(), static_cast<std::streamsize>(header.size()));
  f.write(reinterpret_cast<const char *>(data.data()),
          static_cast<std::streamsize>(data.size() * sizeof(float)));
  return f.good();
}

// ---------------------------------------------------------------------------
// 8-bit normalized depth preview (P5 PGM)
// ---------------------------------------------------------------------------

static bool WriteDepthPreview(const std::string &path,
                              const std::vector<float> &d, int w, int h) {
  float lo = std::numeric_limits<float>::max();
  float hi = std::numeric_limits<float>::lowest();
  for (float v : d) {
    if (v > 1e-4f && std::isfinite(v)) { lo = std::min(lo, v); hi = std::max(hi, v); }
  }
  std::vector<uint8_t> img(d.size(), 0);
  const float range = (hi > lo) ? (hi - lo) : 1.0f;
  for (size_t i = 0; i < d.size(); ++i) {
    if (d[i] > 1e-4f && std::isfinite(d[i])) {
      const float t = (d[i] - lo) / range;
      img[i] = static_cast<uint8_t>(std::clamp(t, 0.0f, 1.0f) * 255.0f);
    }
  }
  return WritePgm(path, img, w, h);
}

// ---------------------------------------------------------------------------
// Colored binary PLY (float x/y/z + uchar r/g/b).
// Disparity math is byte-for-byte the RTP sample's / main.cc's.
// ---------------------------------------------------------------------------

static bool WritePly(const std::string &path,
                     const std::vector<float> &depth,
                     int depth_w, int depth_h,
                     const uint8_t *rgb, int rgb_w, int rgb_h, int rgb_channels,
                     Intrinsics K, bool disparity,
                     double baseline_m, double scale,
                     double min_depth, double max_depth) {
  std::vector<float> xyz;
  std::vector<uint8_t> col;
  xyz.reserve(depth.size() * 3 / 2);
  col.reserve(depth.size() * 3 / 2);

  for (int v = 0; v < depth_h; ++v) {
    for (int u = 0; u < depth_w; ++u) {
      const float val = depth[static_cast<size_t>(v) * depth_w + u];
      if (!std::isfinite(val)) continue;

      double x, y, z;
      if (disparity) {
        if (val < 0.0001f) continue;
        const double t = scale * baseline_m / val;
        z = -t * K.fx;
        x =  t * (u - K.cx);
        y =  t * -(v - K.cy);
      } else {
        if (val < 1e-4f) continue;
        const double zc = static_cast<double>(val) * scale;
        z = -zc;
        const double t = zc / K.fx;
        x = t * (u - K.cx);
        y = t * -(v - K.cy);
      }
      if (std::abs(z) < min_depth || std::abs(z) > max_depth) continue;

      xyz.push_back(static_cast<float>(x));
      xyz.push_back(static_cast<float>(y));
      xyz.push_back(static_cast<float>(z));

      const int cu = (rgb_w == depth_w) ? u : u * rgb_w / depth_w;
      const int cv = (rgb_h == depth_h) ? v : v * rgb_h / depth_h;
      const size_t idx = (static_cast<size_t>(cv) * rgb_w + cu) * rgb_channels;
      if (rgb_channels >= 3) {
        col.push_back(rgb[idx + 0]);
        col.push_back(rgb[idx + 1]);
        col.push_back(rgb[idx + 2]);
      } else {
        col.push_back(rgb[idx]);
        col.push_back(rgb[idx]);
        col.push_back(rgb[idx]);
      }
    }
  }

  const size_t count = xyz.size() / 3;
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << "ply\n"
    << "format binary_little_endian 1.0\n"
    << "element vertex " << count << "\n"
    << "property float x\nproperty float y\nproperty float z\n"
    << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
    << "end_header\n";
  for (size_t i = 0; i < count; ++i) {
    f.write(reinterpret_cast<const char *>(&xyz[i * 3]), 3 * sizeof(float));
    f.write(reinterpret_cast<const char *>(&col[i * 3]), 3);
  }
  return f.good();
}

// ---------------------------------------------------------------------------
// Zero-padded frame stem (e.g. "captures/000042")
// ---------------------------------------------------------------------------

static std::string FrameStem(int64_t idx, const std::string &dir) {
  std::ostringstream ss;
  ss << dir << "/" << std::setw(6) << std::setfill('0') << idx;
  return ss.str();
}

// ---------------------------------------------------------------------------
// Write camera_info.json (once)
// ---------------------------------------------------------------------------

static bool g_info_written = false;

static void WriteCameraInfo(const std::string &output_dir,
                            int img_w, int img_h,
                            int depth_w, int depth_h,
                            const Intrinsics &K_calib) {
  if (g_info_written) return;
  const FrameSize calib_sz{CALIB_W, CALIB_H};
  const FrameSize img_sz{img_w, img_h};
  const FrameSize depth_sz{depth_w, depth_h};
  const Intrinsics Ki = ScaleIntrinsics(K_calib, calib_sz, img_sz);
  const Intrinsics Kd = ScaleIntrinsics(K_calib, calib_sz, depth_sz);

  std::ostringstream j;
  j << std::setprecision(10);
  j << "{\n"
    << "  \"baseline_m\": " << g_baseline_m << ",\n"
    << "  \"depth_scale\": " << g_depth_scale << ",\n"
    << "  \"depth_is_disparity\": " << (g_depth_is_disparity ? "true" : "false") << ",\n"
    << "  \"calibration_resolution\": [" << CALIB_W << ", " << CALIB_H << "],\n"
    << "  \"intrinsics_at_calibration\": {\"fx\": " << K_calib.fx
    << ", \"fy\": " << K_calib.fy << ", \"cx\": " << K_calib.cx
    << ", \"cy\": " << K_calib.cy << "},\n"
    << "  \"image_resolution\": [" << img_w << ", " << img_h << "],\n"
    << "  \"intrinsics_at_image\": {\"fx\": " << Ki.fx << ", \"fy\": " << Ki.fy
    << ", \"cx\": " << Ki.cx << ", \"cy\": " << Ki.cy << "},\n"
    << "  \"depth_resolution\": [" << depth_w << ", " << depth_h << "],\n"
    << "  \"intrinsics_at_depth\": {\"fx\": " << Kd.fx << ", \"fy\": " << Kd.fy
    << ", \"cx\": " << Kd.cx << ", \"cy\": " << Kd.cy << "}\n"
    << "}\n";

  std::ofstream f(output_dir + "/camera_info.json");
  if (f) {
    f << j.str();
    g_info_written = true;
    std::cout << "Wrote camera_info.json" << std::endl;
  }
}

// ---------------------------------------------------------------------------
// HTTP health event-stream subscriber (unchanged from sample_original.cc)
// ---------------------------------------------------------------------------

static void subscribe(const std::string &addr) {
  auto client = httplib::Client(addr);
  auto res = client.Get("/health", [&](const char *data, size_t len) {
    (void)data;
    (void)len;
    // Uncomment to view event stream data:
    // try {
    //   auto body = nlohmann::json::parse(std::string(data, len));
    //   std::cout << body.dump(2) << std::endl;
    // } catch (...) {}
    return should_stay_connected.load();
  });
  if (res && res->status != 200) {
    std::cerr << "Failed to GET /health: status " << res->status << std::endl;
  }
}

// ---------------------------------------------------------------------------
// HTTP helpers for listing server resources (use simple string ops — no JSON)
// ---------------------------------------------------------------------------

static std::string http_get_body(httplib::Client &client, const std::string &path) {
  auto res = client.Get(path.c_str());
  if (!res || res->status != 200) return "";
  return res->body;
}

// ---------------------------------------------------------------------------
// Print usage
// ---------------------------------------------------------------------------

static void PrintUsage(const char *prog) {
  std::cout << "Start USB stream:       " << prog << " <svc-address> --USB <left-bus-id> <right-bus-id> <calibration-filename> [options]\n"
            << "Start DWVO stream:      " << prog << " <svc-address> --DWVO <dwvo-filename> <calibration-filename> [options]\n"
            << "List calibration files: " << prog << " <svc-address> -lc\n"
            << "List DWVO files:        " << prog << " <svc-address> -lv\n"
            << "List device bus IDs:    " << prog << " <svc-address> -ld\n"
            << "\nFile dump options (append after required args):\n"
            << "  --output-dir <dir>        Directory to write frames (default: captures)\n"
            << "  --save-every <N>          Save every Nth frame (default: 1)\n"
            << "  --num-frames <N>          Stop after N saved frames (0 = unlimited)\n"
            << "  --baseline <m>            Stereo baseline in metres (default: 0.1)\n"
            << "  --depth-scale <s>         Output unit scale (default: 100 = m→cm)\n"
            << "  --min-depth <d>           Min depth in output units (default: 0)\n"
            << "  --max-depth <d>           Max depth in output units (default: 1000)\n"
            << "  --depth-is-disparity 0|1  Whether buffer holds disparity (default: 1)\n"
            << "  --no-ply                  Skip PLY writing\n"
            << "  --no-images               Skip left/right .ppm writing\n"
            << "  --no-depth-npy            Skip .npy writing\n"
            << "  --no-depth-preview        Skip depth preview .pgm writing\n";
}

// ---------------------------------------------------------------------------
// Simple command-line flag parser (no absl dependency)
// ---------------------------------------------------------------------------

static bool ParseFlag(int &i, int argc, char **argv,
                      const std::string &flag, std::string &value) {
  if (i < argc && argv[i] == flag) {
    if (i + 1 < argc) { value = argv[i + 1]; i += 2; return true; }
    else { std::cerr << "Missing value for " << flag << std::endl; }
  }
  return false;
}

static bool ParseFlagBool(int &i, int argc, char **argv,
                          const std::string &flag, bool &value) {
  std::string s;
  if (ParseFlag(i, argc, argv, flag, s)) {
    value = (s == "1" || s == "true" || s == "yes");
    return true;
  }
  return false;
}

static bool ParseFlagDouble(int &i, int argc, char **argv,
                            const std::string &flag, double &value) {
  std::string s;
  if (ParseFlag(i, argc, argv, flag, s)) {
    value = std::stod(s);
    return true;
  }
  return false;
}

static bool ParseFlagInt(int &i, int argc, char **argv,
                         const std::string &flag, int &value) {
  std::string s;
  if (ParseFlag(i, argc, argv, flag, s)) {
    value = std::stoi(s);
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  if (argc == 1) { PrintUsage(argv[0]); return 0; }

  g_svc_address = argv[1];
  std::string flag = argv[2];

  // Quick listing commands — no streaming
  std::string addr = g_svc_address + ":" + std::to_string(HTTP_PORT);
  auto client = httplib::Client(addr);

  if (flag == "-lc") {
    std::string body = http_get_body(client, "/list_calibrations");
    std::cout << (body.empty() ? "(empty or error)" : body) << std::endl;
    return body.empty() ? 1 : 0;
  }
  if (flag == "-lv") {
    std::string body = http_get_body(client, "/recordings");
    std::cout << (body.empty() ? "(empty or error)" : body) << std::endl;
    return body.empty() ? 1 : 0;
  }
  if (flag == "-ld") {
    std::string body = http_get_body(client, "/list_devices");
    std::cout << (body.empty() ? "(empty or error)" : body) << std::endl;
    return body.empty() ? 1 : 0;
  }

  // --- Parse mode-specific positional args ---------------------------------
  if (flag == "--USB") {
    if (argc < 6) {
      std::cerr << "Missing inputs for --USB. Usage:" << std::endl;
      PrintUsage(argv[0]);
      return 1;
    }
    g_left_bus_id  = argv[3];
    g_right_bus_id = argv[4];
    g_calib_file   = argv[5];
    g_input_type   = "USB";

    if (g_left_bus_id == g_right_bus_id) {
      std::cerr << "Left and right bus IDs must be unique." << std::endl;
      return 1;
    }
  } else if (flag == "--DWVO") {
    if (argc < 5) {
      std::cerr << "Missing inputs for --DWVO. Usage:" << std::endl;
      PrintUsage(argv[0]);
      return 1;
    }
    g_dwvo_file  = argv[3];
    g_calib_file = argv[4];
    g_input_type = "DWVO";
  } else {
    std::cerr << "Unknown flag: " << flag << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }

  // --- Parse optional file-dump flags (anywhere after positional args) ------
  int pos = (flag == "--USB") ? 6 : 5;
  for (int i = pos; i < argc; ) {
    bool consumed = false;
    consumed |= ParseFlag    (i, argc, argv, "--output-dir",          g_output_dir);
    consumed |= ParseFlagInt (i, argc, argv, "--save-every",          g_save_every);
    consumed |= ParseFlagInt (i, argc, argv, "--num-frames",          g_num_frames);
    consumed |= ParseFlagDouble(i, argc, argv, "--baseline",          g_baseline_m);
    consumed |= ParseFlagDouble(i, argc, argv, "--depth-scale",       g_depth_scale);
    consumed |= ParseFlagDouble(i, argc, argv, "--min-depth",         g_min_depth);
    consumed |= ParseFlagDouble(i, argc, argv, "--max-depth",         g_max_depth);
    consumed |= ParseFlagBool(i, argc, argv, "--depth-is-disparity",  g_depth_is_disparity);
    if (!consumed && i < argc && std::strcmp(argv[i], "--no-ply") == 0) {
      g_write_ply = false; ++i; consumed = true;
    }
    if (!consumed && i < argc && std::strcmp(argv[i], "--no-images") == 0) {
      g_write_images = false; ++i; consumed = true;
    }
    if (!consumed && i < argc && std::strcmp(argv[i], "--no-depth-npy") == 0) {
      g_write_depth_npy = false; ++i; consumed = true;
    }
    if (!consumed && i < argc && std::strcmp(argv[i], "--no-depth-preview") == 0) {
      g_write_depth_preview = false; ++i; consumed = true;
    }
    if (!consumed) {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      ++i;
    }
  }

  // --- Resolve image size from depth mode (matches sample_original.cc) ------
  {
    // Try to detect depth mode from server or use a default.
    // We default to SGM (mode 0) unless the user's calibration/config says otherwise.
    // The actual DEPTH_MODE is set server-side; we match the expected sizes here.
    // In practice the server sends frames sized for its configured mode —
    // we detect this from the actual payload size at runtime (see loop below).
    g_depth_mode = 0;   // default; will be auto-detected
    g_img_w = 800;
    g_img_h = 600;
  }

  // --- Signal handlers ------------------------------------------------------
  std::signal(SIGINT,  [](int) { should_stay_connected = false; should_poll = false; });
  std::signal(SIGTERM, [](int) { should_stay_connected = false; should_poll = false; });

  // --- Create output directory ----------------------------------------------
  {
    std::error_code ec;
    std::filesystem::create_directories(g_output_dir, ec);
    if (ec) {
      std::cerr << "Cannot create output dir " << g_output_dir << ": "
                << ec.message() << std::endl;
      return 1;
    }
  }

  // --- HTTP: subscribe to /health and POST /start ---------------------------
  std::thread subscribe_thread(subscribe, addr);

  // Build the /start request (hand-rolled JSON — no nlohmann dependency).
  std::ostringstream req;
  req << "{"
      << "\"calibration_path\":\"" << g_calib_file << "\","
      << "\"depth_mode\":\"SGM\","
      << "\"input_dwvo\":\"" << g_dwvo_file << "\","
      << "\"port\":" << RTP_PORT << ","
      << "\"resolution\":\"UXGA\","
      << "\"input_type\":\"" << g_input_type << "\","
      << "\"input_left\":\"" << g_left_bus_id << "\","
      << "\"input_right\":\"" << g_right_bus_id << "\","
      << "\"swap_inputs\":false,"
      << "\"gpu_id\":0,"
      << "\"fps\":30"
      << "}";

  std::cout << "Starting stream..." << std::endl;
  // auto res = client.Post("/start", req.str(), "application/json");
  // if (!res) {
  //   std::cerr << "Connection refused. Is RTPSender running on " << g_svc_address << "?"
  //             << std::endl;
  //   should_stay_connected = false;
  //   should_poll = false;
  // } else if (res->status != 200) {
  //   std::cerr << "POST /start error: " << res->body << std::endl;
  //   should_stay_connected = false;
  //   should_poll = false;
  // } else {
  //   // Parse intrinsics from the JSON response (hand-rolled parser — tiny subset).
  //   const std::string &body = res->body;
  //   auto find_val = [&](const std::string &key) -> float {
  //     auto pos = body.find("\"" + key + "\"");
  //     if (pos == std::string::npos) return 0.f;
  //     pos = body.find(':', pos);
  //     if (pos == std::string::npos) return 0.f;
  //     // skip ':' and whitespace
  //     while (++pos < body.size() && (body[pos] == ' ' || body[pos] == '\t' || body[pos] == '\n')) {}
  //     std::string num;
  //     while (pos < body.size() && (std::isdigit(body[pos]) || body[pos] == '.' ||
  //            body[pos] == '-' || body[pos] == '+' || body[pos] == 'e' || body[pos] == 'E')) {
  //       num.push_back(body[pos++]);
  //     }
  //     return num.empty() ? 0.f : std::stof(num);
  //   };
  //   g_fx_calib = find_val("fx");
  //   g_fy_calib = find_val("fy");
  //   g_cx_calib = find_val("cx");
  //   g_cy_calib = find_val("cy");

  //   // If fy wasn't in the response, copy fx (common for square-pixel cameras).
  //   if (g_fy_calib == 0.f) g_fy_calib = g_fx_calib;

  //   std::cout << "Server intrinsics: fx=" << g_fx_calib
  //             << " fy=" << g_fy_calib
  //             << " cx=" << g_cx_calib
  //             << " cy=" << g_cy_calib << std::endl;
  // }
  g_fx_calib = 958.5324097f;
  g_fy_calib = 957.6196899f;
  g_cx_calib = 795.965332f;
  g_cy_calib = 645.6810303f;
  std::cout << "Manually input intrinsics: fx=" << g_fx_calib
              << " fy=" << g_fy_calib
              << " cx=" << g_cx_calib
              << " cy=" << g_cy_calib << std::endl;

  // --- uvgRTP receiver setup ------------------------------------------------
  uvgrtp::context ctx;
  uvgrtp::session *sess = ctx.create_session("0.0.0.0");
  uvgrtp::media_stream *receiver = sess->create_stream(
      RTP_PORT, RTP_FORMAT_GENERIC, RCE_RECEIVE_ONLY | RCE_FRAGMENT_GENERIC);

  // --- Main loop: pull frames, dump to disk ---------------------------------
  std::cout << "Waiting for incoming packets. Press Ctrl+C to stop." << std::endl;
  std::cout << "Output dir: " << g_output_dir
            << " | save_every=" << g_save_every
            << " | depth_is_disparity=" << g_depth_is_disparity << std::endl;

  int64_t frame_index    = 0;
  int64_t saved_count    = 0;
  bool    warned_no_right = false;
  auto    start_time     = std::chrono::steady_clock::now();

  while (should_poll) {
    auto frm = receiver->pull_frame(5000);
    if (!frm) {
      std::cout << "Frame pull timed out. Trying again..." << std::endl;
      continue;
    }

    ++frame_index;

    // --- Auto-detect image size from first frame's payload -----------------
    // Payload = NTP_HEADER + left_img + [right_img?] + depth
    // We know depth is 2*W*H bytes (FP16).  For a 3-channel image:
    //   payload = 8 + W*H*3 + [W*H*3] + W*H*2
    // Try common sizes: 800x600 (SGM), 400x300 (refined_fast), 800x600 (refined_pro)
    if (frame_index == 1) {
      const size_t plen = frm->payload_len;
      // Try to determine W,H from payload size.
      // SGM 800x600:   8 + 800*600*3       + 800*600*2        = 8+1440000+960000 = 2400008
      // ML 400x300:    8 + 400*300*3       + 400*300*2        = 8+360000+240000  = 600008
      // ML 800x600:    8 + 800*600*3       + 800*600*2        = 2400008
      // SGM 800x600+r: 8 + 2*800*600*3     + 800*600*2        = 8+2880000+960000 = 3840008
      const size_t data_len = plen - NTP_HEADER_SIZE;
      // depth = W*H*2, img = W*H*3 per image
      // Try 800x600 SGM (1 img): data = 800*600*3 + 800*600*2 = 2,400,000
      if (data_len == 2400000)      { g_img_w = 800; g_img_h = 600; }
      else if (data_len == 600008)  { g_img_w = 400; g_img_h = 300; }
      else if (data_len == 3840008) { g_img_w = 800; g_img_h = 600; } // +right img
      else if (data_len == 960008)  { g_img_w = 400; g_img_h = 300; } // +right img
      else {
        // Generic fallback: try 800x600
        std::cerr << "Warning: unexpected payload size " << plen
                  << " — assuming 800x600 SGM." << std::endl;
        g_img_w = 800; g_img_h = 600;
      }
      std::cout << "Detected image size: " << g_img_w << "x" << g_img_h << std::endl;
    }

    const int IMG_SIZE  = g_img_w * g_img_h * 3;
    const int DISP_SIZE = g_img_w * g_img_h * 2;
    const size_t min_payload = NTP_HEADER_SIZE + IMG_SIZE + DISP_SIZE;
    const size_t full_payload = NTP_HEADER_SIZE + 2 * IMG_SIZE + DISP_SIZE;

    if (frm->payload_len < min_payload) {
      std::cerr << "Received invalid frame of size " << frm->payload_len
                << " (min expected " << min_payload << ")" << std::endl;
      (void)uvgrtp::frame::dealloc_frame(frm);
      continue;
    }

    const bool has_right = (frm->payload_len >= full_payload);

    // --- Extract buffers from payload ---------------------------------------
    const uint8_t *left_img = frm->payload + NTP_HEADER_SIZE;
    const uint8_t *right_img = has_right ? (left_img + IMG_SIZE) : nullptr;
    const uint8_t *disp_raw  = left_img + (has_right ? 2 * IMG_SIZE : IMG_SIZE);

    // --- Decode disparity/depth to float32 ----------------------------------
    std::vector<float> depth = DecodeDepth(disp_raw, DISP_SIZE, g_img_w, g_img_h);
    if (depth.empty()) {
      std::cerr << "Failed to decode depth buffer." << std::endl;
      (void)uvgrtp::frame::dealloc_frame(frm);
      continue;
    }

    // --- Throttle by save_every ---------------------------------------------
    if (g_save_every > 1 && (frame_index % g_save_every) != 0) {
      (void)uvgrtp::frame::dealloc_frame(frm);
      continue;
    }

    // --- Write camera_info.json on first saved frame ------------------------
    const Intrinsics K_calib{static_cast<double>(g_fx_calib),
                             static_cast<double>(g_fy_calib),
                             static_cast<double>(g_cx_calib),
                             static_cast<double>(g_cy_calib)};
    WriteCameraInfo(g_output_dir, g_img_w, g_img_h, g_img_w, g_img_h, K_calib);

    const std::string stem = FrameStem(saved_count, g_output_dir);

    // --- Write left.ppm -----------------------------------------------------
    if (g_write_images) {
      if (!WritePpmRgb(stem + "_left.ppm", left_img, g_img_w, g_img_h, 3)) {
        std::cerr << "Failed to write " << stem << "_left.ppm" << std::endl;
      }
    }

    // --- Write right.ppm (if available) -------------------------------------
    if (g_write_images && right_img) {
      if (!WritePpmRgb(stem + "_right.ppm", right_img, g_img_w, g_img_h, 3)) {
        std::cerr << "Failed to write " << stem << "_right.ppm" << std::endl;
      }
    } else if (g_write_images && !right_img && !warned_no_right) {
      std::cerr << "Note: RTP payload does not include right image "
                << "(payload_len=" << frm->payload_len
                << ", need " << full_payload << "). "
                << "Skipping right.ppm." << std::endl;
      warned_no_right = true;
    }

    // --- Write depth.npy ----------------------------------------------------
    if (g_write_depth_npy && !depth.empty()) {
      if (!WriteNpyFloat32(stem + "_depth.npy", depth, g_img_h, g_img_w)) {
        std::cerr << "Failed to write " << stem << "_depth.npy" << std::endl;
      }
    }

    // --- Write depth_preview.pgm --------------------------------------------
    if (g_write_depth_preview && !depth.empty()) {
      if (!WriteDepthPreview(stem + "_depth_preview.pgm", depth, g_img_w, g_img_h)) {
        std::cerr << "Failed to write " << stem << "_depth_preview.pgm" << std::endl;
      }
    }

    // --- Write .ply ---------------------------------------------------------
    if (g_write_ply && !depth.empty()) {
      const Intrinsics K_depth = ScaleIntrinsics(
          K_calib,
          FrameSize{CALIB_W, CALIB_H},
          FrameSize{g_img_w, g_img_h});
      if (!WritePly(stem + ".ply", depth, g_img_w, g_img_h,
                    left_img, g_img_w, g_img_h, 3,
                    K_depth, g_depth_is_disparity,
                    g_baseline_m, g_depth_scale,
                    g_min_depth, g_max_depth)) {
        std::cerr << "Failed to write " << stem << ".ply" << std::endl;
      }
    }

    // --- Progress reporting -------------------------------------------------
    ++saved_count;
    if (saved_count == 1 || saved_count % 30 == 0) {
      const double secs = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - start_time)
                              .count();
      std::cerr << "Saved " << saved_count << " frames ("
                << (saved_count / secs) << " saves/s)" << std::endl;
    }

    if (g_num_frames > 0 && saved_count >= g_num_frames) {
      should_poll = false;
      should_stay_connected = false;
    }

    (void)uvgrtp::frame::dealloc_frame(frm);
  }

  // --- Cleanup --------------------------------------------------------------
  std::cout << "Cleaning up resources... ";
  subscribe_thread.join();
  std::cout << "done." << std::endl;
  std::cout << "Wrote " << saved_count << " frames to " << g_output_dir << "/"
            << std::endl;
  return 0;
}
