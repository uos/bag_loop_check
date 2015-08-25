from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()

d['packages'] = ['bag_loop_check']
d['package_dir'] = {'':'src'}
d['install_requires'] = []

setup(**d)
