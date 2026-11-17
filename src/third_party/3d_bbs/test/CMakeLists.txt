add_executable(voxelmaps_saver src/voxelmaps_saver.cpp)
target_link_libraries(voxelmaps_saver PRIVATE cpu_bbs3d Boost::filesystem Boost::system)
set_target_properties(voxelmaps_saver PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS voxelmaps_saver RUNTIME DESTINATION bin)

if(BUILD_OPENCL AND BUILD_BBS_OPENCL_TEST)
  add_executable(opencl_test src/opencl_test.cpp)
  target_link_libraries(opencl_test PRIVATE opencl_bbs3d)
  set_target_properties(opencl_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    INSTALL_RPATH "$ORIGIN/../lib")
  install(TARGETS opencl_test RUNTIME DESTINATION bin)
endif()
