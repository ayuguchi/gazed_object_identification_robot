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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/root/catkin_ws/src/image_pipeline/camera_calibration"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/root/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/root/catkin_ws/install/lib/python2.7/dist-packages:/root/catkin_ws/build/camera_calibration/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/root/catkin_ws/build/camera_calibration" \
    "/usr/bin/python" \
    "/root/catkin_ws/src/image_pipeline/camera_calibration/setup.py" \
    build --build-base "/root/catkin_ws/build/camera_calibration" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/root/catkin_ws/install" --install-scripts="/root/catkin_ws/install/bin"
