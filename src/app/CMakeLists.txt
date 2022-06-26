qt_wrap_ui(MSLAM_VIEWER_UIS viewer/ui/viewer_main_window.ui)


add_executable(
  mslam_viewer
  ${MSLAM_VIEWER_UIS}
  viewer/viewer.cpp
  viewer/viewer_main_window.cpp
)

target_include_directories(mslam_viewer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(
  mslam_viewer PRIVATE Qt6::Widgets ${realsense2_LIBRARY} modular_slam
  opencv_core
  opencv_video
  opencv_imgproc
  opencv_highgui
)
