# Options are "release" and "debug" (see CMakePresets.json)
BUILD_TYPE=${BUILD_TYPE:-release}

git submodule sync vcpkg
cd vcpkg && ./bootstrap-vcpkg.sh
cd ..

cmake --build --preset ninja-vcpkg-${BUILD_TYPE}
