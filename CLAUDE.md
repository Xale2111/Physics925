# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Template for the GPR925 physics module (SAE Geneva): each student writes a 2D physics engine in C++23 and shows it in samples that run on Windows, Linux and the Nintendo DS (devkitPro/devkitARM, ARM946E-S, no FPU) from the same code. Layout:
- `externals/EngineLib`: git submodule (branch `main`), the students' shared maths library (namespace `maths`, flat includes such as `"vector2.h"`). Don't edit it from this repo.
- `include/gpr925`, `src`: `Gpr925Physics`, the engine library. It starts with `fixed.h` (`gpr925::Fixed`, libnds' 20.12 `f32` in a class), `scalar.h` (`Scalar` and its helpers) and `vec2.h` (`Vec2`, `Rect`).
- `main/samples/`: `Gpr925Samples`, the platform-neutral samples (no SDL, ImGui or libnds). A sample simulates only in `FixedUpdate(const Input&)` and describes itself in a world-space `DrawList` (`draw_list.h`, 16 palette colors). Sizes come from `sample_config.h` (per platform, `GPR925_PLATFORM_NDS`). Samples are registered in the `SampleRegistry` constructor (`sample.cc`).
- `main/sdl/`: the PC host (`Gpr925Sample` exe): SDL3 window, ImGui menu, fixed-step accumulator, `SdlSampleDrawer`.
- `nds/`: the DS ROM (`Gpr925DsSamples`): `NdsSampleDrawer` (hardware sprites plus a BG3 8bpp bitmap), menu and timings console on the bottom screen.
- `tests/`: GoogleTest executables, one per file, linked to `Gpr925Samples` and `GTest::gtest_main`.

## Scalar

`gpr925::Scalar` is chosen by the CMake cache variable `GPR925_SCALAR`: `FLOAT` (PC presets) or `FIXED` (`nds` and `debug-fixed` presets). Everything outside `fixed.h` must build and pass the tests with both.

- Portable code uses `Scalar`, `Vec2`, the operators and the helpers `Ratio`, `ToScalar`, `ToInt`, `ToFloat`, `Sqrt`, `Abs`. No float literal in run-time expressions, no `std::sqrt`.
- `Fixed`'s constructor from `float` is `consteval`: constants only.
- `Fixed` division and `Sqrt` use the DS hardware divider and square root unit (libnds `divf32`, `sqrtf32`): never from an interrupt handler. On PC and in constant evaluation they use portable code that gives the same bits.
- `Vec2` mirrors the members of `maths::Vector2` (component-wise `*`, no scalar product) so that it can become `using Vec2 = maths::Vector2<Scalar>;` once EngineLib's `Number` concept accepts user-defined types. Keep code that uses `Vec2` within that set of members.
- With `FIXED`, nothing on the DS path may link soft-float: check with `arm-none-eabi-nm build/nds/nds/Gpr925DsSamples.elf | grep -E '__aeabi_(f|d)'` and print with `iprintf`/`sniprintf`.

## Build

```
cmake --preset debug          # float; needs VCPKG_ROOT
cmake --build --preset debug
ctest --preset debug
cmake --preset debug-fixed    # same with GPR925_SCALAR=FIXED
cmake --preset nds            # DS ROM (build/nds/nds/Gpr925DsSamples.nds); needs DEVKITPRO, no vcpkg
cmake --build --preset nds
```

## Style

[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html):
- `.clang-format` (Google, 80 columns) formats everything outside `externals/`. Run `clang-format -i` on touched files.
- `.clang-tidy` checks `google-*` and Google naming: `snake_case` variables and fields, `snake_case_` private members, `kCamelCase` constants and enumerators, `CamelCase` functions. Run `run-clang-tidy -p build/debug -extra-arg=-Wno-unknown-warning-option '(src|main|tests)/'`.
- Sources are `.cc`. Header guards are named from the path (`GPR925_FIXED_H_`, `MAIN_SAMPLES_SAMPLE_H_`). Includes name the full path: `"gpr925/..."` for the engine, the repo-root path for the rest (`"main/samples/sample.h"`).
- Public symbols in `include/gpr925` and `main/samples` headers get a `///` doc comment written for game programming students (concepts, units, caveats). Implementation comments are `//`.
- No exceptions or RTTI in the engine, the samples and the DS host (`-fno-exceptions -fno-rtti`); the tests keep them.

## Warnings

`cmake/CompilerWarnings.cmake` defines `gpr925_set_project_warnings(<target>)`; `gpr925_set_engine_options(<target>)` (top-level `CMakeLists.txt`) adds the no-exceptions and DS flags. New targets must call one of them. `GPR925_WARNINGS_AS_ERRORS` is ON, and EngineLib's own warning flags are `INTERFACE`, so they also apply to every target that links it. Clean with GCC 16 and devkitARM 16 (2026-10-13); not yet built with MSVC.
