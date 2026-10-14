# Platform-neutral samples: scenes that fill a DrawList, with no SDL, ImGui or libnds. Rendered by a host: the SDL
# executable on PC (main/sdl/), the DS ROM (nds/).
file(GLOB SAMPLE_FILES CONFIGURE_DEPENDS *.cc *.h)
add_library(Gpr925Samples STATIC ${SAMPLE_FILES})
# Hosts include "main/samples/sample.h"
target_include_directories(Gpr925Samples PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(Gpr925Samples PUBLIC Gpr925Physics)
gpr925_set_engine_options(Gpr925Samples)
set_target_properties(Gpr925Samples PROPERTIES FOLDER Samples/)
