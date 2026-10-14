# DS samples ROM: NdsSampleDrawer renders the platform-neutral Gpr925Samples on the top screen (hardware sprites and a
# BG3 8bpp bitmap), with the sample menu and the timings on the bottom screen.
# Only built with devkitPro's NDS toolchain (the nds preset).
add_executable(Gpr925DsSamples main.cc nds_sample_drawer.cc nds_sample_drawer.h)
target_link_libraries(Gpr925DsSamples PRIVATE Gpr925Samples)
target_link_options(Gpr925DsSamples PRIVATE -Wl,-Map,$<TARGET_FILE_DIR:Gpr925DsSamples>/Gpr925DsSamples.map)
gpr925_set_engine_options(Gpr925DsSamples)
nds_create_rom(Gpr925DsSamples NAME "GPR925 Physics" SUBTITLE1 "Samples" SUBTITLE2 "SAE Geneva")
