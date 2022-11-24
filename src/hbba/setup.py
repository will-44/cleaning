from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    # packages=['script'],
    packages=['hbba_common'],
    package_dir={'': 'script'}
)
setup(**d)