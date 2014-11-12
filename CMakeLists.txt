cmake_minimum_required(VERSION 2.8)
project( Ex2 )

#
find_package( OpenCV REQUIRED )

#
add_executable( Ex2 
	util.cpp
	harris.cpp

	ex2.cpp
)


target_include_directories( Ex2
	PRIVATE 	.
)

target_link_libraries( Ex2 
	${OpenCV_LIBS} 
)
