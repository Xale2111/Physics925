//
// Created by alexk on 21.11.2026.
//

#ifndef GPR925PHYSICS_INCLUDE_GPR925_PROFILER_H_
#define GPR925PHYSICS_INCLUDE_GPR925_PROFILER_H_

#ifdef GPR925_USE_TRACY
  #include <tracy/Tracy.hpp>
  // Unique variable name per line: ZoneScoped always uses the same name, so a
  // zone inside another zone would trigger -Wshadow (and -Werror stops the build)
  #define GPR925_CAT_(a, b) a##b
  #define GPR925_CAT(a, b) GPR925_CAT_(a, b)
  #define GPR925_ZONE() ZoneNamed(GPR925_CAT(gpr925_zone_, __LINE__), true)
  #define GPR925_ZONE_NAMED(name) \
    ZoneNamedN(GPR925_CAT(gpr925_zone_, __LINE__), name, true)
  #define GPR925_FRAME() FrameMark
  #define GPR925_PLOT(name, value) \
    TracyPlot(name, value)  // int, float or double, not Scalar
#else
  #define GPR925_ZONE()
  #define GPR925_ZONE_NAMED(name)
  #define GPR925_FRAME()
  #define GPR925_PLOT(name, value)
#endif

#endif  // GPR925PHYSICS_INCLUDE_GPR925_PROFILER_H_
