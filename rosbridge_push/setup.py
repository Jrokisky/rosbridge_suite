from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'rosbridge_push',
    ],
    package_dir={'': 'src'}
)


setup(**d)
