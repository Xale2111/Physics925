// PC sample host: SDL window, ImGui sample menu, fixed-step accumulator. The
// samples themselves are platform-neutral (Gpr925Samples) and are rendered
// through SdlSampleDrawer.
#include <imgui.h>

#include <cstdint>
#include <cstdio>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_render.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "main/samples/input.h"
#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "main/sdl/sdl_sample_drawer.h"

namespace {

// Longest frame time fed to the accumulator: after a stall (a breakpoint, a
// window drag) the simulation slows down instead of running hundreds of steps
// to catch up
constexpr float kMaxFrameTime = 0.25f;
constexpr float kFixedDeltaTime = 1.0f / 60.0f;

struct KeyBinding {
  gpr925::Button button;
  SDL_Scancode key;
  SDL_GamepadButton gamepad_button;
};

// Scancodes are key positions, not the letters printed on them. E, S, D and F
// have the same position and the same letter on US (QWERTY), Swiss (QWERTZ)
// and French (AZERTY) keyboards, which is not the case of W, A, Z, Q, Y or M.
// The action buttons keep the diamond of the DS on the keyboard and on the
// gamepad (by position: an Xbox pad prints A on the south button).
constexpr KeyBinding kBindings[] = {
    {gpr925::Button::kUp, SDL_SCANCODE_UP, SDL_GAMEPAD_BUTTON_DPAD_UP},
    {gpr925::Button::kDown, SDL_SCANCODE_DOWN, SDL_GAMEPAD_BUTTON_DPAD_DOWN},
    {gpr925::Button::kLeft, SDL_SCANCODE_LEFT, SDL_GAMEPAD_BUTTON_DPAD_LEFT},
    {gpr925::Button::kRight, SDL_SCANCODE_RIGHT, SDL_GAMEPAD_BUTTON_DPAD_RIGHT},
    {gpr925::Button::kA, SDL_SCANCODE_F, SDL_GAMEPAD_BUTTON_EAST},
    {gpr925::Button::kB, SDL_SCANCODE_D, SDL_GAMEPAD_BUTTON_SOUTH},
    {gpr925::Button::kX, SDL_SCANCODE_E, SDL_GAMEPAD_BUTTON_NORTH},
    {gpr925::Button::kY, SDL_SCANCODE_S, SDL_GAMEPAD_BUTTON_WEST},
};

// Buttons of the virtual pad that are down, from the keyboard and the gamepad
// (which may be null)
std::uint8_t ReadHeldButtons(SDL_Gamepad* gamepad) {
  const bool* keys = SDL_GetKeyboardState(nullptr);
  const bool use_keyboard = !ImGui::GetIO().WantCaptureKeyboard;
  unsigned held = 0;
  for (const auto& binding : kBindings) {
    const bool key_down = use_keyboard && keys[binding.key];
    const bool pad_down = gamepad != nullptr &&
                          SDL_GetGamepadButton(gamepad, binding.gamepad_button);
    if (key_down || pad_down) {
      held |= gpr925::ButtonMask(binding.button);
    }
  }
  return static_cast<std::uint8_t>(held);
}

void DrawSampleMenu(gpr925::SampleRegistry& registry,
                    const gpr925::SampleContext& context) {
  ImGui::Begin("Sample Manager");
  const auto current = registry.current_index();
  if (ImGui::BeginCombo("Samples", current == gpr925::SampleRegistry::kNone
                                       ? "None"
                                       : registry.name(current))) {
    if (ImGui::Selectable("None", current == gpr925::SampleRegistry::kNone)) {
      registry.Select(gpr925::SampleRegistry::kNone, context);
    }
    for (std::size_t index = 0; index < registry.size(); index++) {
      if (ImGui::Selectable(registry.name(index), current == index)) {
        registry.Select(index, context);
      }
    }
    ImGui::EndCombo();
  }
  if (current != gpr925::SampleRegistry::kNone && ImGui::Button("Restart")) {
    registry.Select(current, context);
  }
  ImGui::TextUnformatted("Arrows: D-pad   F: A   D: B   E: X   S: Y");
  ImGui::End();
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
    std::printf("error initializing SDL: %s\n", SDL_GetError());
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow("GPR925 Physics Samples", 1280, 720,
                                        SDL_WINDOW_RESIZABLE);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
  if (window == nullptr || renderer == nullptr) {
    std::printf("error creating the window: %s\n", SDL_GetError());
    return 1;
  }
  SDL_SetRenderVSync(renderer, 1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer3_Init(renderer);

  SDL_Gamepad* gamepad = nullptr;
  {
    gpr925::SampleRegistry registry;
    const gpr925::SampleContext context{gpr925::sample_config::kWorldBounds};
    registry.Select(0, context);
    gpr925::DrawList draw_list;
    draw_list.Reserve(gpr925::sample_config::kMaxDrawCircles,
                      gpr925::sample_config::kMaxDrawBoxes,
                      gpr925::sample_config::kMaxDrawLines);
    gpr925::SdlSampleDrawer drawer{renderer};

    float accumulator = 0.0f;
    std::uint8_t previous_held = 0;
    auto last_tick = SDL_GetPerformanceCounter();
    bool is_closed = false;
    while (!is_closed) {
      const auto current_tick = SDL_GetPerformanceCounter();
      const float frame_time =
          static_cast<float>(current_tick - last_tick) /
          static_cast<float>(SDL_GetPerformanceFrequency());
      last_tick = current_tick;

      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL3_ProcessEvent(&event);
        if (event.type == SDL_EVENT_QUIT) {
          is_closed = true;
        } else if (event.type == SDL_EVENT_GAMEPAD_ADDED &&
                   gamepad == nullptr) {
          gamepad = SDL_OpenGamepad(event.gdevice.which);
        } else if (event.type == SDL_EVENT_GAMEPAD_REMOVED &&
                   gamepad != nullptr &&
                   SDL_GetGamepadID(gamepad) == event.gdevice.which) {
          SDL_CloseGamepad(gamepad);
          gamepad = nullptr;
        }
      }

      ImGui_ImplSDLRenderer3_NewFrame();
      ImGui_ImplSDL3_NewFrame();
      ImGui::NewFrame();
      DrawSampleMenu(registry, context);

      const std::uint8_t held = ReadHeldButtons(gamepad);
      accumulator += frame_time < kMaxFrameTime ? frame_time : kMaxFrameTime;
      while (accumulator >= kFixedDeltaTime) {
        // "pressed" is only set on the first step after the button went down
        registry.FixedUpdate(gpr925::Input::FromHeld(held, previous_held));
        previous_held = held;
        accumulator -= kFixedDeltaTime;
      }

      SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
      SDL_RenderClear(renderer);
      registry.Draw(draw_list);
      drawer.Render(draw_list);

      ImGui::Render();
      ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
      SDL_RenderPresent(renderer);
    }
  }

  if (gamepad != nullptr) {
    SDL_CloseGamepad(gamepad);
  }
  ImGui_ImplSDLRenderer3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
