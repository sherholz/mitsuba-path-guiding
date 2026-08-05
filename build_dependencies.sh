mkdir build
mkdir build/deps
cd build/deps

export CMAKE_POLICY_VERSION_MINIMUM=3.5

cmake   -DBUILD_DEPENDENCIES_ONLY=ON \
        -DBUILD_OIDN=ON \
        -DBUILD_OIDN_FROM_SOURCE=ON \
        -DBUILD_TBB=ON \
        -DBUILD_TBB_FROM_SOURCE=ON \
        -DBUILD_OPENPGL=OFF \
        -DCMAKE_INSTALL_PREFIX=../../deps \
        ../../superbuild
make -j 
cd ..
rm -rdf build/deps