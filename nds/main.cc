// DS samples ROM: the Gpr925Samples on the top screen (NdsSampleDrawer), a
// menu and timings on the bottom screen console. One fixed step per VBlank
// (60 Hz = Sample::kFixedDeltaTime), no accumulator.
//
// Everything printed goes through newlib's integer-only iprintf/sniprintf, so
// the float printf code (and the soft-float routines it pulls in) stays out of
// the ROM.
#include <nds.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "main/samples/input.h"
#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "nds/nds_sample_drawer.h"

namespace {

constexpr int kStatsWindow = 60;
// The frame period is measured with calico's system tick counter (tickGetCount,
// TICK_FREQ = bus clock / 64). Hardware timers 2 and 3 run that counter
// (cpuStartTiming reads it too), so they must not be reprogrammed.
constexpr std::uint64_t kTickHzX10 = std::uint64_t{TICK_FREQ} * 10;
// VBlanks per second (59.8261 Hz) x 10
constexpr std::uint32_t kVblankHzX10 = 598;

constexpr int kConsoleColumns = 32;
constexpr int kMenuRow = 2;
// Samples shown at once in the menu: it scrolls with the selection
constexpr std::size_t kMenuRowCount = 7;
constexpr int kStatsRow = 10;
constexpr int kHelpRow = 20;

struct KeyBinding {
  gpr925::Button button;
  std::uint32_t key;
};

// L, R and START are kept for the menu
constexpr KeyBinding kBindings[] = {
    {gpr925::Button::kUp, KEY_UP},     {gpr925::Button::kDown, KEY_DOWN},
    {gpr925::Button::kLeft, KEY_LEFT}, {gpr925::Button::kRight, KEY_RIGHT},
    {gpr925::Button::kA, KEY_A},       {gpr925::Button::kB, KEY_B},
    {gpr925::Button::kX, KEY_X},       {gpr925::Button::kY, KEY_Y},
};

// Buttons of the virtual pad among libnds key bits (keysHeld, keysDown)
std::uint8_t ToButtons(std::uint32_t keys) {
  unsigned buttons = 0;
  for (const auto& binding : kBindings) {
    if ((keys & binding.key) != 0) {
      buttons |= gpr925::ButtonMask(binding.button);
    }
  }
  return static_cast<std::uint8_t>(buttons);
}

struct TimingWindow {
  std::uint32_t total = 0;
  std::uint32_t max = 0;
  int count = 0;

  void Add(std::uint32_t ticks) {
    total += ticks;
    max = ticks > max ? ticks : max;
    ++count;
  }
  [[nodiscard]] std::uint32_t AverageUs() const {
    return count == 0
               ? 0
               : timerTicks2usec(total / static_cast<std::uint32_t>(count));
  }
  [[nodiscard]] std::uint32_t MaxUs() const { return timerTicks2usec(max); }
};

// Values shown on the console, refreshed every kStatsWindow frames
struct DisplayedStats {
  std::uint32_t step_average_us = 0;
  std::uint32_t step_max_us = 0;
  std::uint32_t draw_average_us = 0;
  std::uint32_t draw_max_us = 0;
  std::uint32_t fps_x10 = 0;
};

// Console rows as last printed: a row is only reprinted when its text changes,
// to keep console time down
class ConsoleRows {
 public:
  __attribute__((format(printf, 3, 4))) void Print(int row, const char* format,
                                                   ...) {
    std::array<char, kConsoleColumns + 1> line{};
    va_list args;
    va_start(args, format);
    vsniprintf(line.data(), line.size(), format, args);
    va_end(args);
    // Pad to the full width, so a shorter text erases the previous one
    const std::size_t length = std::strlen(line.data());
    std::memset(line.data() + length, ' ', kConsoleColumns - length);
    line[kConsoleColumns] = '\0';
    auto& previous = rows_[static_cast<std::size_t>(row)];
    if (std::strcmp(previous.data(), line.data()) != 0) {
      previous = line;
      // The last column is left empty: printing into it would wrap the cursor
      // to the next row
      iprintf("\x1b[%d;0H%.31s", row, line.data());
    }
  }

 private:
  std::array<std::array<char, kConsoleColumns + 1>, 24> rows_{};
};

}  // namespace

int main() {
  gpr925::NdsSampleDrawer drawer;
  drawer.Init();

  videoSetModeSub(MODE_0_2D);
  vramSetBankC(VRAM_C_SUB_BG);
  PrintConsole console;
  consoleInit(&console, 0, BgType_Text4bpp, BgSize_T_256x256, 31, 0, false,
              true);
  consoleSelect(&console);

  gpr925::SampleRegistry registry;
  const gpr925::SampleContext context{gpr925::sample_config::kWorldBounds};
  gpr925::DrawList draw_list;
  draw_list.Reserve(gpr925::sample_config::kMaxDrawCircles,
                    gpr925::sample_config::kMaxDrawBoxes,
                    gpr925::sample_config::kMaxDrawLines);
  registry.Select(0, context);

  ConsoleRows rows;
  rows.Print(0, "GPR925 Physics samples");
  rows.Print(kHelpRow, "L/R: sample  START: restart");
  rows.Print(kHelpRow + 1, "D-pad, A, B, X, Y: the sample");

  TimingWindow step_timing;
  TimingWindow draw_timing;
  std::uint64_t window_frame_ticks = 0;
  DisplayedStats displayed;
  std::uint64_t last_frame_tick = tickGetCount();

  while (pmMainLoop()) {
    scanKeys();
    const auto keys = keysDown();
    const std::size_t sample_count = registry.size();
    if ((keys & (KEY_L | KEY_R)) != 0 && sample_count > 1) {
      const std::size_t current = registry.current_index();
      registry.Select((keys & KEY_L) != 0
                          ? (current + sample_count - 1) % sample_count
                          : (current + 1) % sample_count,
                      context);
      step_timing = {};
      draw_timing = {};
      window_frame_ticks = 0;
    }
    if ((keys & KEY_START) != 0) {
      registry.Select(registry.current_index(), context);
    }

    // One step per VBlank: keysDown is exactly "went down on this step"
    const gpr925::Input input{ToButtons(keysHeld()), ToButtons(keys)};
    cpuStartTiming(0);
    registry.FixedUpdate(input);
    step_timing.Add(cpuEndTiming());

    cpuStartTiming(0);
    registry.Draw(draw_list);
    drawer.Render(draw_list);
    draw_timing.Add(cpuEndTiming());

    // The menu shows kMenuRowCount samples around the selected one
    const std::size_t current = registry.current_index();
    const std::size_t last_first =
        sample_count > kMenuRowCount ? sample_count - kMenuRowCount : 0;
    const std::size_t first =
        std::min(current > kMenuRowCount / 2 ? current - kMenuRowCount / 2 : 0,
                 last_first);
    for (std::size_t row = 0; row < kMenuRowCount; ++row) {
      const std::size_t index = first + row;
      if (index < sample_count) {
        rows.Print(kMenuRow + static_cast<int>(row), "%c %s",
                   index == current ? '>' : ' ', registry.name(index));
      } else {
        rows.Print(kMenuRow + static_cast<int>(row), " ");
      }
    }
    const auto& stats = drawer.stats();
    rows.Print(kStatsRow, "Shapes  %3zu circles %2zu boxes",
               draw_list.circles.size(), draw_list.boxes.size());
    rows.Print(kStatsRow + 1, "Lines   %3zu", draw_list.lines.size());
    rows.Print(kStatsRow + 2, "Sprites %3d used %3d dropped",
               stats.sprites_used, stats.dropped_sprites);
    rows.Print(kStatsRow + 3, "Bitmap  %3d shapes", stats.bitmap_shapes);
    rows.Print(kStatsRow + 4, "Redraws %" PRIu32, stats.bitmap_redraws);

    swiWaitForVBlank();
    drawer.Present();

    const std::uint64_t frame_tick = tickGetCount();
    window_frame_ticks += frame_tick - last_frame_tick;
    last_frame_tick = frame_tick;
    if (step_timing.count == kStatsWindow) {
      displayed.step_average_us = step_timing.AverageUs();
      displayed.step_max_us = step_timing.MaxUs();
      displayed.draw_average_us = draw_timing.AverageUs();
      displayed.draw_max_us = draw_timing.MaxUs();
      displayed.fps_x10 =
          window_frame_ticks == 0
              ? 0
              : static_cast<std::uint32_t>(kTickHzX10 * kStatsWindow /
                                           window_frame_ticks);
      step_timing = {};
      draw_timing = {};
      window_frame_ticks = 0;
    }
    rows.Print(kStatsRow + 6, "Step %5" PRIu32 " us avg %5" PRIu32 " max",
               displayed.step_average_us, displayed.step_max_us);
    rows.Print(kStatsRow + 7, "Draw %5" PRIu32 " us avg %5" PRIu32 " max",
               displayed.draw_average_us, displayed.draw_max_us);
    // VBlanks per frame x 10, rounded
    const std::uint32_t vblanks_per_frame_x10 =
        displayed.fps_x10 == 0
            ? 0
            : (kVblankHzX10 * 10 + displayed.fps_x10 / 2) / displayed.fps_x10;
    rows.Print(kStatsRow + 8,
               "FPS  %2" PRIu32 ".%" PRIu32 " (%" PRIu32 ".%" PRIu32
               " VBlank/frame)",
               displayed.fps_x10 / 10, displayed.fps_x10 % 10,
               vblanks_per_frame_x10 / 10, vblanks_per_frame_x10 % 10);
  }
  return 0;
}
