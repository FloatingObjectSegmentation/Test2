define all_libs
	-L$(1) \
	$(shell ls $(1)/$(2) | grep dylib | sed 's/.*\/lib\([^\/]*\).dylib/-l\1/')
endef

INCLUDES := -I/usr/local/include/eigen3/Eigen \
            -I/usr/local/include/pcl-1.8/pcl \
			-I/usr/local/include/vtk-8.1
LIBS := $(call all_libs,/usr/local/Cellar/pcl/1.8.1_2/lib,libpcl*.1.8.1.dylib) \
		$(call all_libs,/usr/local/Cellar/vtk/8.1.0/lib,libvtk*8.1.1.dylib) \
        -L/usr/local/Cellar/boost/1.66.0/lib \
			-lboost_system \
			-lboost_thread-mt
		#$(call all_libs,/usr/local/Cellar/boost/1.66.0/lib,*.dylib)

all: main
	./main data/regiongrowingdata.pcd

main: main.cpp Makefile
	clang++ -std=c++11 -o $@ ${INCLUDES} ${LIBS} $<