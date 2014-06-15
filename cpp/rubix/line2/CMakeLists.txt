cmake_minimum_required(VERSION 2.8)
project(line2)
find_package(OpenCV REQUIRED)
file(GLOB line2_SRC
	"*.h"
	"*.cpp"
)
add_executable(line2 ${line2_SRC})
target_link_libraries(line2 ${OpenCV_LIBS})
