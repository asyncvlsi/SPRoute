set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-unused-but-set-variable -Wno-maybe-uninitialized -Wno-unused-function")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable -Wno-unused-but-set-variable -Wno-maybe-uninitialized -Wno-unused-function")

set(LEF_INCLUDE /net/ohm/export/cdgc/michael/EDA/lef/lef)
#set(LEF_INCLUDE /net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/include/lef56)
set(DEF_INCLUDE /net/ohm/export/cdgc/michael/EDA/def/def)
#set(OPENDB_INCLUDE /net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/include/opendb)

include_directories(${DEF_INCLUDE} ${LEF_INCLUDE} ${OPENDB_INCLUDE})

link_directories(/usr/lib64)
 
app(lefdef_SPRoute main.cpp parserDataType.cpp defDataBase.cpp lefDataBase.cpp  dist.c dl.c err.c heap.c mst2.c neighbors.c bookshelf_IO.c memAlloc.c
EXTLIBS /net/ohm/export/cdgc/michael/EDA/def/lib/libdef.a
#/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/lef56/libopendblef.a
		/net/ohm/export/cdgc/michael/EDA/lef/lib/liblef.a
		/net/ohm/export/cdgc/michael/boost-1.61.0/lib/libboost_serialization.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/defin/libdefin.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/lefin/liblefin.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/db/libopendb.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/zutil/libzutil.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/zlib/libzlib.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/lefout/liblefout.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/defout/libdefout.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/swig/tcl/libopendbtcl.a
        #/net/ohm/export/cdgc/michael/EDA/OpenRoad/OpenDB/build/src/tm/libtm.a
        #/usr/lib64/libtcl.so
)
target_link_libraries(lefdef_SPRoute -Wno-unused-variable)
