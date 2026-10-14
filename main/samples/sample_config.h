#ifndef MAIN_SAMPLES_SAMPLE_CONFIG_H_
#define MAIN_SAMPLES_SAMPLE_CONFIG_H_

#include <cstddef>

#include "gpr925/vec2.h"

namespace gpr925 {

/// Per-platform sample sizes: world bounds, object counts and draw capacities.
///
/// The values depend on `GPR925_PLATFORM_NDS`: the DS keeps every sample
/// under its 128 hardware sprites and its 4 MB of RAM, the PC shows bigger
/// scenes. A sample reads its sizes from here rather than hard-coding them,
/// so the same code runs on both. Add one constant per platform for each of
/// your samples.
namespace sample_config {
#if defined(GPR925_PLATFORM_NDS)
/// World rectangle shown by the hosts, in meters.
///
/// 12.8 x 9.6 m on the DS: 20 px/m on the 256x192 top screen.
inline constexpr Rect kWorldBounds{{Scalar{}, Scalar{}},
                                   {Ratio(64, 5), Ratio(48, 5)}};
/// Number of bouncing balls in @ref HelloSample.
inline constexpr std::size_t kHelloBallCount = 24;
/// @ref DrawList circle capacity reserved by the hosts.
inline constexpr std::size_t kMaxDrawCircles = 128;
/// @ref DrawList box capacity reserved by the hosts.
inline constexpr std::size_t kMaxDrawBoxes = 16;
/// @ref DrawList line capacity reserved by the hosts.
inline constexpr std::size_t kMaxDrawLines = 64;
#else
/// World rectangle shown by the hosts, in meters.
///
/// 12.8 x 7.2 m on PC: 100 px/m in a 1280x720 window. 12.8 x 9.6 m on the DS:
/// 20 px/m on the 256x192 top screen.
inline constexpr Rect kWorldBounds{{Scalar{}, Scalar{}},
                                   {Ratio(64, 5), Ratio(36, 5)}};
/// Number of bouncing balls in @ref HelloSample (24 on the DS).
inline constexpr std::size_t kHelloBallCount = 200;
/// @ref DrawList circle capacity reserved by the hosts (128 on the DS).
inline constexpr std::size_t kMaxDrawCircles = 1024;
/// @ref DrawList box capacity reserved by the hosts (16 on the DS).
inline constexpr std::size_t kMaxDrawBoxes = 64;
/// @ref DrawList line capacity reserved by the hosts (64 on the DS).
inline constexpr std::size_t kMaxDrawLines = 512;
#endif
}  // namespace sample_config

}  // namespace gpr925

#endif  // MAIN_SAMPLES_SAMPLE_CONFIG_H_
