#!/bin/bash
FILE_libigl=$(pwd)/include/dependencies/libigl
if [ ! -d $FILE_libigl ]; then
    mkdir -p $FILE_libigl
    git clone https://github.com/libigl/libigl.git $FILE_libigl
fi
FILE_eigen=$(pwd)/include/dependencies/eigen-3.4.0
if [ ! -d $FILE_eigen ]; then    
    wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip -P $(pwd)/include/dependencies
    unzip $(pwd)/include/dependencies/eigen*.zip -d $(pwd)/include/dependencies
    rm -rf $(pwd)/include/dependencies/eigen*.zip
fi
FILE_mex=/usr/local/bin/mex
if [ ! -f $FILE_mex ]; then
    mex_file=$(find / -name "mex" | grep "bin/mex" )
    if (( $(echo "${mex_file}" | grep -c "mex") > 0 )); then
        mex_file_1=$(echo "${mex_file}" | head -1)
        ln -s $mex_file_1 $FILE_mex
        mex include/cpp/src/ray_tracing_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/libigl/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp"
        mex include/cpp/src/ray_tracing_no_thres_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/libigl/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp"
        mex include/cpp/src/CURL_extraction_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp -std=c++17" LDFLAGS="\$LDFLAGS -fopenmp"
        mex include/cpp/src/CURL_reconstruction_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp -std=c++17" LDFLAGS="\$LDFLAGS -fopenmp"
        mv *.mexa64 include
    else
        echo "Can't find 'mex' command, please insure that Matlab has been installed or link mex command by yourself 'sudo ln -s <path>/mex /usr/local/bin/mex'"
    fi
else
    mex include/cpp/src/ray_tracing_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/libigl/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp"
    mex include/cpp/src/ray_tracing_no_thres_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/libigl/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp"
    mex include/cpp/src/CURL_extraction_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp -std=c++17" LDFLAGS="\$LDFLAGS -fopenmp"
    mex include/cpp/src/CURL_reconstruction_mex.cpp -Iinclude/cpp/include -Iinclude/dependencies/eigen-3.4.0 CXXFLAGS="\$CXXFLAGS -fopenmp -std=c++17" LDFLAGS="\$LDFLAGS -fopenmp"
    mv *.mexa64 include
fi