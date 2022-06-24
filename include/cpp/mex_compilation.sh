#!/bin/bash
gitclone 

mex src/ray_tracing_mex.cpp -Iinclude -I/home/zkc/Application/libigl/include -I/home/zkc/Application/eigen-3.4.0/installation/include/eigen3 CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp"

mex src/ray_tracing_no_thres_mex.cpp -Iinclude -I/home/zkc/Application/libigl/include -I/home/zkc/Application/eigen-3.4.0/installation/include/eigen3 CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp"

mex src/CURL_extraction_mex.cpp -Iinclude -I/home/zkc/Application/eigen-3.4.0/installation/include/eigen3 CXXFLAGS="\$CXXFLAGS -fopenmp -std=c++17" LDFLAGS="\$LDFLAGS -fopenmp"

mex src/CURL_reconstruction_mex.cpp -Iinclude -I/home/zkc/Application/eigen-3.4.0/installation/include/eigen3 CXXFLAGS="\$CXXFLAGS -fopenmp -std=c++17" LDFLAGS="\$LDFLAGS -fopenmp"

mv *.mexa64 mex_files