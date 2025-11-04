#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jetson/agv_tester/src/vision_opencv/image_geometry"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jetson/agv_tester/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jetson/agv_tester/install/lib/python3/dist-packages:/home/jetson/agv_tester/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jetson/agv_tester/build" \
    "/usr/bin/python3" \
    "/home/jetson/agv_tester/src/vision_opencv/image_geometry/setup.py" \
    egg_info --egg-base /home/jetson/agv_tester/build/vision_opencv/image_geometry \
    build --build-base "/home/jetson/agv_tester/build/vision_opencv/image_geometry" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jetson/agv_tester/install" --install-scripts="/home/jetson/agv_tester/install/bin"
