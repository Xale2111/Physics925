# Physics925

Template for the GPR925 physics module: write your own 2D physics engine and
show it in samples that run on **Windows, Linux and the Nintendo DS** from the
same code.

A sample simulates in `FixedUpdate`, then describes itself in a `DrawList`
(circles, boxes and lines in world space). The hosts do the rest: an SDL3
window with an ImGui menu on PC, hardware sprites and a bitmap on the DS.

## Layout

| Folder | Content |
|---|---|
| `externals/EngineLib` | The shared maths library (`maths::Vector2`...), a git submodule |
| `include/gpr925`, `src` | `Gpr925Physics`: **your engine goes here**. Starts with `Fixed`, `Scalar` and `Vec2` |
| `main/samples` | `Gpr925Samples`: the platform-neutral samples. **Your samples go here** |
| `main/sdl` | PC host: `Gpr925Sample` executable |
| `nds` | DS host: `Gpr925DsSamples.nds` ROM |
| `tests` | GoogleTest executables, one per file |

## Get the code

Create your repository with **Use this template** on GitHub, then clone it
with its submodule (you need access to the EngineLib repository and an SSH key
on GitHub):

```
git clone --recurse-submodules git@github.com:<you>/<your-repo>.git
```

If `externals/EngineLib` is empty: `git submodule update --init --recursive`.
To get the latest EngineLib: `git submodule update --remote externals/EngineLib`.

## Build on PC

You need CMake 3.25+, Ninja, a recent C++23 compiler (MSVC, GCC or Clang) and
[vcpkg](https://github.com/microsoft/vcpkg), with the `VCPKG_ROOT` environment
variable set to its folder. vcpkg downloads SDL3, ImGui and GoogleTest on the
first configure.

```
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
build/debug/Gpr925Sample
```

In CLion, open the folder and enable the `debug` preset in *Settings > Build >
CMake*.

Controls: arrow keys for the D-pad, and the DS diamond on `E` (X), `S` (Y),
`D` (B), `F` (A): keys that are at the same place on US, Swiss and French
keyboards. A gamepad works too, with the buttons at their DS positions.

## Build for the Nintendo DS

Install [devkitPro](https://devkitpro.org/wiki/Getting_Started) with the
`nds-dev` group and set the `DEVKITPRO` environment variable (`/opt/devkitpro`
on Linux, `C:/devkitPro` on Windows). vcpkg is not used.

```
cmake --preset nds
cmake --build --preset nds
```

Open `build/nds/nds/Gpr925DsSamples.nds` in [melonDS](https://melonds.kuribo64.net/).
The top screen shows the sample, the bottom screen the menu and the time taken
by `FixedUpdate` and by the drawing, in microseconds.

Controls: D-pad, A and B for the sample; L/R to change sample, START to
restart it.

## Presets

| Preset | `Scalar` | Use |
|---|---|---|
| `debug`, `release`, `relwithdebinfo` | `float` | PC |
| `debug-fixed` | `Fixed` | PC, to debug the DS behaviour with a real debugger |
| `nds` | `Fixed` | DS ROM |

## Numbers: `Scalar`, `Fixed` and `Vec2`

The DS has no FPU, so `float` is emulated in software and is very slow.
`gpr925::Scalar` (`include/gpr925/scalar.h`) is `float` on PC and
`gpr925::Fixed` on the DS.

`gpr925::Fixed` (`include/gpr925/fixed.h`) wraps libnds' 20.12 fixed-point
`f32`: `*` calls `mulf32`, `/` calls `divf32` and `Sqrt` calls `sqrtf32`. One
unit is 4096, the smallest step is 1/4096 and the range is about ±524 287.

To write code that builds everywhere:

- use `Scalar` and `Vec2`, never `float`;
- write constants as `Ratio(981, 100)` or `constexpr Scalar kHalf{0.5f}`, not
  as float literals in the middle of an expression;
- use `Sqrt`, `Abs`, `ToInt`, `ToScalar` instead of `std::sqrt`, casts...

The simulation is not bit-identical between `float` and `Fixed`.

`gpr925::Vec2` is a small struct with the same members as EngineLib's
`maths::Vector2`. It exists because `maths::Vector2<gpr925::Fixed>` does not
compile: EngineLib's `Number` concept only accepts built-in types. Once
EngineLib accepts user-defined number types, replace it in
`include/gpr925/vec2.h` with `using Vec2 = maths::Vector2<Scalar>;`.

## Add a sample

1. Copy `main/samples/01_hello.h` and `.cc` to `02_my_sample.h` and `.cc`,
   and rename the class.
2. Put its sizes for each platform in `main/samples/sample_config.h`.
3. Register it in the `SampleRegistry` constructor in
   `main/samples/sample.cc`.
4. Add a test for it in `tests/samples_test.cc`.

There is nothing to change in `main/sdl` or `nds`. See the documentation of
`Sample` in `main/samples/sample.h` for the rules.

On the DS, at most 128 small shapes (circles up to 32 px of radius, boxes up
to 64x64 px) are hardware sprites; extra ones are dropped. Bigger shapes and
lines are drawn in a bitmap, which is slower.

## Style

Google C++ style, checked by `.clang-format` and `.clang-tidy`. Warnings are
errors on every platform (`GPR925_WARNINGS_AS_ERRORS`).

```
clang-format -i <files>
run-clang-tidy -p build/debug -extra-arg=-Wno-unknown-warning-option '(src|main|tests)/'
```
