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

echo_and_run cd "/home/jcosta/workspace/ROS/crawler/src/crawler_roboclaw/roboclaw_node"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jcosta/workspace/ROS/crawler/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jcosta/workspace/ROS/crawler/install/lib/python3/dist-packages:/home/jcosta/workspace/ROS/crawler/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jcosta/workspace/ROS/crawler/build" \
    "/usr/bin/python3" \
    "/home/jcosta/workspace/ROS/crawler/src/crawler_roboclaw/roboclaw_node/setup.py" \
     \
    build --build-base "/home/jcosta/workspace/ROS/crawler/build/crawler_roboclaw/roboclaw_node" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jcosta/workspace/ROS/crawler/install" --install-scripts="/home/jcosta/workspace/ROS/crawler/install/bin"
