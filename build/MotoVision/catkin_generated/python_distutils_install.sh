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

echo_and_run cd "/home/calum/MotoWorkspace/src/MotoVision"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/calum/MotoWorkspace/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/calum/MotoWorkspace/install/lib/python2.7/dist-packages:/home/calum/MotoWorkspace/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/calum/MotoWorkspace/build" \
    "/usr/bin/python" \
    "/home/calum/MotoWorkspace/src/MotoVision/setup.py" \
    build --build-base "/home/calum/MotoWorkspace/build/MotoVision" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/calum/MotoWorkspace/install" --install-scripts="/home/calum/MotoWorkspace/install/bin"
