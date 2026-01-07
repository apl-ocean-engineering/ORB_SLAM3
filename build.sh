BUILD_TYPE=${BUILD_TYPE:-release}

git submodule sync vcpkg
cd vcpkg && ./bootstrap-vcpkg.sh
cd ..

cmake --build --preset ninja-vcpkg-${BUILD_TYPE}


echo "Uncompress vocabulary ..."
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..
