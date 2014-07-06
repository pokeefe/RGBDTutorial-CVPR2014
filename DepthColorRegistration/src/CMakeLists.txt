find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_executable (DepthColorRegistration main.cpp Registration.cpp)

target_link_libraries(DepthColorRegistration 
  ${OpenCV_LIBRARIES}
  )
