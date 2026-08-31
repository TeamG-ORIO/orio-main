"""orio_core — pure, unit-testable core library for the ORIO project.

Everything in this package is *honest*: functions take all inputs as arguments
(including any random-number generator or clock they need), return values, and
perform no ROS I/O, hardware access, disk access, logging, or other side effects.
This is what makes the package importable with no ROS running and testable with
plain pytest.

The dishonest edges (rospy pub/sub, FrankaArm motion, service calls, Tkinter,
file/pickle I/O, seeding the RNG) live in the ROS node scripts that import this
library — never in here.
"""

from . import robot_util  # noqa: F401
from . import perception_geometry  # noqa: F401
from . import planning  # noqa: F401

__all__ = ["robot_util", "perception_geometry", "planning"]
