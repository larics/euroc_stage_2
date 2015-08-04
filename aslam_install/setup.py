
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['argparse_config', 'aslam_install_util'],
    package_dir={'':'bin'},
    scripts=['bin/aslam_status','bin/aslam-create-pkg']
)

setup(**setup_args)
