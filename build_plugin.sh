g++ -std=c++17 -shared -fPIC \
    -I/usr/lib/x86_64-linux-gnu/pkgconfig/../../..//include/gazebo-11 \
    -I/usr/lib/x86_64-linux-gnu/pkgconfig/../../..//include/sdformat-9.10 \
    -I/usr/lib/x86_64-linux-gnu/pkgconfig/../../..//include/ignition/math6 \
    -I/usr/lib/x86_64-linux-gnu/pkgconfig/../../..//include/ignition/common3 \
    -I/usr/include/OGRE \
    -I/usr/include/OGRE/Terrain \
    -I/usr/lib/x86_64-linux-gnu/pkgconfig/../../..//include/ignition/transport8 \
    -I/usr/lib/x86_64-linux-gnu/pkgconfig/../../..//include/ignition/fuel_tools4 \
    -I/usr/lib/x86_64-linux-gnu/pkgconfig/../../..//include/ignition/msgs5 \
    RandomMovementPlugin.cpp \
    -o plugins/libRandomMovementPlugin.so