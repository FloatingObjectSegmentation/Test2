define all_libs
	-L$(1) \
	$(shell ls $(1)/$(2) | grep dylib | sed 's/.*\/lib\([^\/]*\).dylib/-l\1/')
endef

INCLUDES := -Iinclude \
			-I/usr/local/include/eigen3/Eigen \
            -I/usr/local/include/pcl-1.8/pcl \
			-I/usr/local/include/vtk-8.1

LIBS := $(call all_libs,/usr/local/Cellar/pcl/1.8.1_2/lib,libpcl*.1.8.1.dylib) \
		$(call all_libs,/usr/local/Cellar/vtk/8.1.0/lib,libvtk*8.1.1.dylib) \
		$(call all_libs,/usr/local/Cellar/liblas/1.8.1/lib,liblas*.dylib) \
        -L/usr/local/Cellar/boost/1.66.0/lib \
			-lboost_system \
			-lboost_thread-mt
		#$(call all_libs,/usr/local/Cellar/boost/1.66.0/lib,*.dylib)

SRCS := $(wildcard src/*.cpp)
OBJS := $(SRCS:src/%.cpp=obj/%.o)

all: main
	./main data/GK_533_114.las

	# data/GK_533_114.las
	# data/cat.pcd
	# data/regiongrowingdata.pcd
	# data/region_growing_tutorial.pcd
	# data/region_growing_rgb_tutorial.pcd
	# data/min_cut_segmentation_tutorial.pcd
	

main: main.cpp ${OBJS}
	clang++ -std=c++11 -o $@ ${INCLUDES} ${OBJS} ${LIBS} $<

obj/%.o: src/%.cpp
	clang++ -std=c++11 -c -o $@ ${INCLUDES} $<

format:
	astyle -A1W1HSKNYLt main.cpp