# CMakeList.txt : CMake project for CarDetectionandCountingYOLO, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (CarDetectionandCountingYOLO "CarDetectionandCountingYOLO.cpp" "CarDetectionandCountingYOLO.h" "VehicleCounter.cpp" "Tracker.cpp" "VehicleCounter.h" "Tracker.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET CarDetectionandCountingYOLO PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/third_party/opencv4.10/opencv-4.10.0)

find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR})

include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

target_link_libraries(CarDetectionandCountingYOLO ${OpenCV_LIBRARIES})
