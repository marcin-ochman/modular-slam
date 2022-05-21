add_executable(mslam_viewer viewer/viewer.cpp viewer/viewer_main_window.cpp)
target_include_directories(mslam_viewer PRIVATE include)
target_link_libraries(
  mslam_viewer PRIVATE Qt5::Widgets ${realsense2_LIBRARY} modular_slam
  opencv_core
  opencv_video
  opencv_imgproc
  opencv_highgui
)
