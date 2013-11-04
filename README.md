chipsat
======

CMAKE build system used.

How to build program(s):

Go to project's root folder (on BeagleBone):

Create new build directory:

rm -rf build-test

mkdir build-test

Step into the directory:

cd build-test

Create makefiles using cmake:

cmake .. # will build only the main project

cmake .. -DCMAKE_BUILD_TYPE=Debug # will also build the example programs

make      # Starts the actual compiling process

The created binary files will be located in the bin-directory of the root folder
Recompiling can be done by just calling "make" again in the build-directory the 
cmake steps are only necessary if no build directory is present, or one wants to switch
the build type (i.e. in- or exclude the example projects).
