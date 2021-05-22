cmake_minimum_required(VERSION 3.4.1)

set(pathPROJECT /Users/poiu694/Desktop/File/Detail_AR) # 수정필요
set(pathOPENCV ${pathPROJECT}/sdk)
set(pathLIBOPENCV_JAVA ${pathOPENCV}/native/libs/${ANDROID_ABI}/libopencv_java4.so)

set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

include_directories(${pathOPENCV}/native/jni/include)


add_library( # Sets the name of the library.
        native-lib
        # Sets the library as a shared library.
        SHARED
        # Provides a relative path to your source file(s).
        ${pathPROJECT}/app/src/main/cpp/native-lib.cpp
        ${pathPROJECT}/app/src/main/cpp/Detection.cpp
        ${pathPROJECT}/app/src/main/cpp/basic.h
        ${pathPROJECT}/app/src/main/cpp/function.h)



add_library( lib_opencv SHARED IMPORTED )

set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION ${pathLIBOPENCV_JAVA})



find_library(
        log-lib
        log )

target_link_libraries( # Specifies the target library.
        native-lib
        lib_opencv
        # Links the target library to the log library
        # included in the NDK.
        ${log-lib} )