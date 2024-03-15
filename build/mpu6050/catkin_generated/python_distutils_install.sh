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

echo_and_run cd "/home/shihab/omobot_js/src/mpu6050"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/shihab/omobot_js/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/shihab/omobot_js/install/lib/python2.7/dist-packages:/home/shihab/omobot_js/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/shihab/omobot_js/build" \
    "/usr/bin/python2" \
    "/home/shihab/omobot_js/src/mpu6050/setup.py" \
     \
    build --build-base "/home/shihab/omobot_js/build/mpu6050" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/shihab/omobot_js/install" --install-scripts="/home/shihab/omobot_js/install/bin"
