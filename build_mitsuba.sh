CURRENT_DIR=$PWD
mkdir build
mkdir build/mitsuba
cd build/mitsuba

cmake   -DBUILD_DEPENDENCIES=OFF \
        -DBUILD_DEPENDENCIES_ONLY=OFF \
        -DBUILD_OIDN=OFF \
        -DBUILD_TBB=OFF \
        -DBUILD_OPENPGL=ON \
        -DTBB_ROOT=${CURRENT_DIR}/deps \
        -DDEPS_DIR=${CURRENT_DIR}/deps  \
        -DCMAKE_INSTALL_PREFIX=${CURRENT_DIR}/dist \
        ../../superbuild
make -j 
make -j install
