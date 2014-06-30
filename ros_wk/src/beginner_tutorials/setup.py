from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['beginner_tutorials'],
    package_dir={'':'src'},
    requires=['std_msgs','rospy']
)

setup(**setup_args)
