#!/bin/sh
DEPENDENCIES="../../engine/third-party"
ENGINE_INCLUDES="-I../../engine/sdk -I$DEPENDENCIES/entt/src -I$DEPENDENCIES/spdlog/include -I$DEPENDENCIES/glm -I$DEPENDENCIES/magic_enum/include"
CFLAGS="-ffast-math -ffp-contract=fast -msse4.1 -mfma -mavx2 -mpopcnt"

# Uncomment if this module doesn't include a components.toml
CPPFLAGS="-DNO_COMPONENTS"

MODULE_NAME=physics-physx

MODULE_INCLUDES="-I$DEPENDENCIES/physx/physx/include -I$DEPENDENCIES/physx/pxshared/include"
MODULE_LINKER="-pthread -ldl -lPhysX_static_64 -lPhysXPvdSDK_static_64 -lPhysXExtensions_static_64 -lPhysXCooking_static_64 -lPhysXCommon_static_64 -lPhysXFoundation_static_64"

if [ "$1" = "debug" ]; then
    MODULE_LINKER="$MODULE_LINKER -L$DEPENDENCIES/physx/physx/bin/linux.clang/checked"
    CFLAGS="$CFLAGS -D_DEBUG -DDEBUG_BUILD"
    MODULE_OBJECT=../../engine/build-debug/sdk/module/entry.o
else
    MODULE_LINKER="$MODULE_LINKER -L$DEPENDENCIES/physx/physx/bin/linux.clang/release"
    CFLAGS="$CFLAGS -DNDEBUG"
    MODULE_OBJECT=../../engine/build-release/sdk/module/entry.o
fi

echo Deleting old object files
rm *.o loaders/*.o
for file in *.cpp loaders/*.cpp
do
    echo Building $file
    clang++ -fPIC -std=c++17 $CFLAGS $CPPFLAGS $ENGINE_INCLUDES $MODULE_INCLUDES $INCLUDES $LTO -c $file -o $file.o
done
echo Linking module
clang++ *.o loaders/*.o $MODULE_LINKER -shared $MODULE_OBJECT -o ../physics-physx.module