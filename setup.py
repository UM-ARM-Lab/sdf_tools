## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sdf_tools'],
    package_dir={'': 'src'},
)

setup(requires=['numpy'],
      **setup_args)
