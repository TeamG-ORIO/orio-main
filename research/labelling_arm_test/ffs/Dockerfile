# Fast-FoundationStereo as a ROS1 node, for research/labelling_arm_test.
#
# Built ON TOP of `ffs`, the image the fast-foundationstereo repo's docker/dockerfile
# produces (conda env `my`: Python 3.12, torch 2.6.0+cu124; it needs the xformers pin in
# reference/dockerfile_xformers_pin.patch, see reference/ORIO_QUICKSTART.md). That image
# has no ROS. rospy and the message packages are pure Python and come from rospypi, as in
# orio_bringup/docker/Dockerfile.suction. The fast-foundationstereo code, the checkpoint
# and this directory are mounted by run_ffs_depth.sh, so nothing from the repos is baked in.
#
# Build (no context needed):
#   docker build -t orio/ffs-ros:latest - < research/labelling_arm_test/ffs/Dockerfile
FROM ffs

# The base image ends with SHELL ["/bin/bash", "-c", "source ~/.bashrc && conda activate my"],
# under which a RUN's command string becomes bash's $0 and never executes. Plain bash -c
# here, and the conda env's binaries by absolute path.
SHELL ["/bin/bash", "-c"]

RUN /opt/conda/envs/my/bin/pip install --no-cache-dir \
        --extra-index-url https://rospypi.github.io/simple/ \
        rospy std_msgs sensor_msgs geometry_msgs std_srvs

# rosgraph's RospyLogger.findCaller loops forever on Python >= 3.11 when no frame matches
# (its walk stops advancing at the outermost frame), so the first rospy.loginfo hangs the
# node. Stop the walk there instead; only the caller name in log records is affected.
# (Same patch as Dockerfile.suction; the assert fails the build if rosgraph changed.)
RUN /opt/conda/envs/my/bin/python - <<'PATCH'
import rosgraph.roslogging as m
p = m.__file__
s = open(p).read()
old = """                break
            if f.f_back:
                f = f.f_back
"""
new = """                break
            if not f.f_back:
                break
            f = f.f_back
"""
assert s.count(old) == 1, "rosgraph findCaller changed; re-check the patch"
open(p, "w").write(s.replace(old, new))
PATCH

# rospypi's rospy ships without ROS's python_logging.conf, and without one rospy drops
# every rospy.loginfo/logwarn. This is Noetic's file, streaming to the console (as with
# roslaunch's output="screen") and to the node's log file.
RUN mkdir -p /etc/ros && printf '%s\n' \
        '[loggers]' 'keys=root, rosout' \
        '[handlers]' 'keys=fileHandler,streamHandler' \
        '[formatters]' 'keys=defaultFormatter' \
        '[logger_root]' 'level=INFO' 'handlers=fileHandler' \
        '[logger_rosout]' 'level=INFO' 'handlers=streamHandler' 'propagate=1' 'qualname=rosout' \
        '[handler_fileHandler]' 'class=handlers.RotatingFileHandler' 'level=DEBUG' \
        'formatter=defaultFormatter' "args=(os.environ['ROS_LOG_FILENAME'],'a', 50000000, 4)" \
        '[handler_streamHandler]' 'class=rosgraph.roslogging.RosStreamHandler' 'level=DEBUG' \
        'formatter=defaultFormatter' 'args=(True,)' \
        '[formatter_defaultFormatter]' 'format=[%(name)s][%(levelname)s] %(asctime)s: %(message)s' \
        > /etc/ros/python_logging.conf
ENV ROS_PYTHON_LOG_CONFIG_FILE=/etc/ros/python_logging.conf

ENV MPLBACKEND=Agg
# Detached container: unbuffered, so `docker logs -f` shows lines as they happen.
ENV PYTHONUNBUFFERED=1

CMD ["/opt/conda/envs/my/bin/python", "/opt/labelling_arm_test/ffs/ffs_depth_node.py"]
