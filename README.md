# SProute
SPRoute: A parallel global router using the Galois framework
SPRoute exploits nested parallelism by a novel hybrid algorithm for parallelization. It combines net-level parallelism in which each thread works on a net, and a fine-grain parallelism which route individual nets in parallel. In fine-grain parallelism, parallelization is through exloration of multiple path. 

This is Lef/Def based version.

#Building SPRoute
You can checkout the latest Lef/Def SPRoute by

git clone https://github.com/asyncvlsi/SPRoute.git

#Dependencies

Dependencies of Galois

A modern C++ compiler compliant with the C++-14 standard (GCC >= 6.1, Intel >= 17.0, LLVM >= 4.0)

CMake (>= 3.2.3)

Boost library ( >= 1.58.0, we recommend building/installing the full library) 

Lef/Def Parser v5.8

#Compile

cd [Galois_Home]/lonestar/experimental/

mkdir lefdef_SPRoute

cp files to lefdef_SPRoute

add a line to the [Galois_Home]/lonestar/experimental/CMakeList.txt: add_subdirectory(lefdef_SPRoute)

change the 5 paths in CMake of lefdef_SPRoute

build the whole Galois 

Then you can make in lefdef_SPRoute's build directory


#DOTO: THIS FILE IS STILL UNDER CONSTRUCTION


