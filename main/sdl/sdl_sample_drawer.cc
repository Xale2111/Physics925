#include "main/sdl/sdl_sample_drawer.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <SDL3/SDL_log.h>

#include "gpr925/scalar.h"

namespace gpr925 {

namespace {

constexpr int kMinCircleSegments = 8;
constexpr int kMaxCircleSegments = 32;
// Inside of the camera rectangle, so that the letterbox bars show
constexpr SDL_FColor kBackgroundColor{0.08f, 0.08f, 0.1f, 1.0f};

SDL_FColor ToSdlColor(ColorId color) {
  const auto& c = kSamplePalette[color];
  return {static_cast<float>(c.r) / 255.0f, static_cast<float>(c.g) / 255.0f,
          static_cast<float>(c.b) / 255.0f, 1.0f};
}

}  // namespace

SdlSampleDrawer::Viewport SdlSampleDrawer::FitCamera(const Rect& camera) const {
  int width = 0;
  int height = 0;
  SDL_GetCurrentRenderOutputSize(renderer_, &width, &height);
  const Vec2 camera_size = camera.Size();
  const float camera_width = ToFloat(camera_size.x);
  const float camera_height = ToFloat(camera_size.y);
  if (camera_width <= 0.0f || camera_height <= 0.0f) {
    return {};
  }
  const float scale = std::min(static_cast<float>(width) / camera_width,
                               static_cast<float>(height) / camera_height);
  return {scale,
          (static_cast<float>(width) - camera_width * scale) / 2.0f -
              ToFloat(camera.min_bound.x) * scale,
          (static_cast<float>(height) - camera_height * scale) / 2.0f -
              ToFloat(camera.min_bound.y) * scale};
}

void SdlSampleDrawer::Render(const DrawList& draw_list) {
  const auto viewport = FitCamera(draw_list.camera);

  const Vec2 camera_size = draw_list.camera.Size();
  const SDL_FRect background{
      ToFloat(draw_list.camera.min_bound.x) * viewport.scale +
          viewport.offset_x,
      ToFloat(draw_list.camera.min_bound.y) * viewport.scale +
          viewport.offset_y,
      ToFloat(camera_size.x) * viewport.scale,
      ToFloat(camera_size.y) * viewport.scale};
  SDL_SetRenderDrawColorFloat(renderer_, kBackgroundColor.r, kBackgroundColor.g,
                              kBackgroundColor.b, kBackgroundColor.a);
  SDL_RenderFillRect(renderer_, &background);

  RenderCircles(draw_list, viewport);
  RenderBoxes(draw_list, viewport);
  RenderLines(draw_list, viewport);
}

void SdlSampleDrawer::RenderCircles(const DrawList& draw_list,
                                    const Viewport& viewport) {
  vertices_.clear();
  indices_.clear();
  for (const auto& circle : draw_list.circles) {
    const SDL_FPoint center{
        ToFloat(circle.center.x) * viewport.scale + viewport.offset_x,
        ToFloat(circle.center.y) * viewport.scale + viewport.offset_y};
    const float radius = ToFloat(circle.radius) * viewport.scale;
    const int segments = std::clamp(static_cast<int>(radius),
                                    kMinCircleSegments, kMaxCircleSegments);
    const auto color = ToSdlColor(circle.color);
    const int first = static_cast<int>(vertices_.size());
    vertices_.push_back({center, color, {}});
    for (int i = 0; i < segments; i++) {
      const float angle = 2.0f * std::numbers::pi_v<float> *
                          static_cast<float>(i) / static_cast<float>(segments);
      vertices_.push_back({{center.x + radius * std::cos(angle),
                            center.y + radius * std::sin(angle)},
                           color,
                           {}});
      indices_.push_back(first);
      indices_.push_back(first + 1 + i);
      indices_.push_back(first + 1 + (i + 1) % segments);
    }
  }
  if (!vertices_.empty() &&
      !SDL_RenderGeometry(renderer_, nullptr, vertices_.data(),
                          static_cast<int>(vertices_.size()), indices_.data(),
                          static_cast<int>(indices_.size()))) {
    SDL_Log("SDL_RenderGeometry: %s", SDL_GetError());
  }
}

void SdlSampleDrawer::RenderBoxes(const DrawList& draw_list,
                                  const Viewport& viewport) {
  // Consecutive boxes of the same color go in one SDL_RenderFillRects
  std::size_t begin = 0;
  while (begin < draw_list.boxes.size()) {
    const auto color = draw_list.boxes[begin].color;
    rects_.clear();
    std::size_t end = begin;
    for (; end < draw_list.boxes.size() && draw_list.boxes[end].color == color;
         ++end) {
      const auto& box = draw_list.boxes[end];
      const auto top_left = box.center - box.half_size;
      rects_.push_back(
          {ToFloat(top_left.x) * viewport.scale + viewport.offset_x,
           ToFloat(top_left.y) * viewport.scale + viewport.offset_y,
           ToFloat(box.half_size.x) * 2.0f * viewport.scale,
           ToFloat(box.half_size.y) * 2.0f * viewport.scale});
    }
    SetDrawColor(color);
    SDL_RenderFillRects(renderer_, rects_.data(),
                        static_cast<int>(rects_.size()));
    begin = end;
  }
}

void SdlSampleDrawer::RenderLines(const DrawList& draw_list,
                                  const Viewport& viewport) {
  // SDL_RenderLines draws a connected polyline, so independent segments are
  // drawn one by one, grouped by color
  ColorId current_color = sample_color::kEmpty;
  for (const auto& line : draw_list.lines) {
    if (line.color != current_color) {
      SetDrawColor(line.color);
      current_color = line.color;
    }
    SDL_RenderLine(renderer_,
                   ToFloat(line.a.x) * viewport.scale + viewport.offset_x,
                   ToFloat(line.a.y) * viewport.scale + viewport.offset_y,
                   ToFloat(line.b.x) * viewport.scale + viewport.offset_x,
                   ToFloat(line.b.y) * viewport.scale + viewport.offset_y);
  }
}

void SdlSampleDrawer::SetDrawColor(ColorId color) {
  const auto sdl_color = ToSdlColor(color);
  SDL_SetRenderDrawColorFloat(renderer_, sdl_color.r, sdl_color.g, sdl_color.b,
                              sdl_color.a);
}

}  // namespace gpr925
