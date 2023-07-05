# SPRoute: A Scalable Parallel Global Router

### Build tools
* CMake, version >= 3.16
* GNU Compiler Collection (GCC), version >= 4.8.5

### Pre-requisite libraries
* Boost, version >= 1.61.0
* Si2 LEF/DEF parser: [LEF parser](https://github.com/The-OpenROAD-Project/lef) and [DEF parser](https://github.com/The-OpenROAD-Project/def). Use `cmake .. -DCMAKE_INSTALL_PREFIX=path/to/install` to specify the installation destination for each parser, and export the path as environment variable `LEF_ROOT` and `DEF_ROOT`. This LEF/DEF parser requires BISON (>= 3.0.4)
* [ACT](https://github.com/asyncvlsi/act): environment variable `ACT_HOME` determines the installation destination of this package
* PhyDB: phydb should be installed in 'ACT_HOME': 'ACT_HOME/include' should contain a 'phydb' folder with all phydb headers, 'ACT_HOME/lib' should contain libphydb.a
* Galois: https://github.com/IntelligentSoftwareSystems/Galois, should be installed in ACT_HOME
* POWV.dat and POST.dat: from FLUTE (http://home.eng.iastate.edu/~cnchu/flute.html), shoule be in the working directory

### Required Evironment variable
* 'LEF_ROOT': installation path of the Si2 lef parser
* 'DEF_ROOT': installation path of the Si2 def parser
* 'ACT_HOME': installation path of ACT, where PhyDB and Galois is installed as well 

### Clone, compile, and install
    $ git clone https://github.com/asyncvlsi/SPRoute.git
    $ cd SPRoute/
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ make install
 If you want an executable of SPRoute, run "cmake .. -DBUILD_BIN=1" in the cmake step.
 "make install" creates a folder `sproute` under folder `$ACT_HOME/include` with all .h of PWRoute/include, and a static library `libsproute.a` under folder `$ACT_HOME/lib`.

### Performance
This version of SPRoute differs from the one used for SPRoute 1.0 (ICCAD2019) or 2.0 (ASP-DAC2022). 
If you want to compare the runtime and performance of SPRoute 1.0/2.0, please email @Michael Jiayuan He (hejiayuan19920229@gmail.com) for a standalone version of SPRoute.
