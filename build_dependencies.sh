mkdir build
mkdir build/deps
cd build/deps

cmake   -DBUILD_DEPENDENCIES_ONLY=ON \
        -DBUILD_OIDN=ON \
        -DBUILD_OIDN_FROM_SOURCE=ON \
        -DBUILD_TBB=ON \
        -DBUILD_TBB_FROM_SOURCE=ON \
        -DBUILD_OPENPGL=OFF \
        -DCMAKE_INSTALL_PREFIX=../../deps \
        ../../superbuild
make -j 
make install
cd ..
rm -rdf build/deps