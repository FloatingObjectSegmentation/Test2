## Dependencies

You should install all dependencies using homebrew.

```
brew install eigen
brew install vtk
brew install boost
brew install pcl
brew install liblas
brew install llvm
brew install make
```

## Running

- You may possibly have to modify the Makefile that finds, compiles and links all of the dependencies together.
- To change the input file, you should correct the argument in the Makefile under 

```
all: main 
	./main <input_file>
```

Finally, run the program using ```make```.

## Contents

#### What is it?

It's a simple framework implementing 3 methods of segmentation over 2 types of files containing point clouds.

#### File types supported

- **.pcd** files are natively supported through PCL.
- **.las** files are supported through libLAS.

#### Methods

##### Usage

- All method inherit from the ```Method``` class. The segmentation procedure is run through the ```computeSegmentation``` method.

##### Available methods

- RegionGrowingRGB - Uses color information for clustering instead of normals.
- MinCut
- ProgressiveMorphologicalFiltering - An algorithm fit for .las files. The purpose is to segment out the ground from the scene.