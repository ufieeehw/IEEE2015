import platform
if platform.system() == 'Linux':
    ## ! DO NOT MANUALLY INVOKE THIS portion of setup.py, USE CATKIN INSTEAD
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=['xmega_driver'],
        package_dir={'': 'src'},
    )

    setup(**setup_args)
else:
    ## This runs if we're on windows
    from setuptools import setup
    setup(name='xmega_driver',
        version='1.0',
        description='Python xmega driver',
        author='Jacob Panikulam',
        author_email='jpanikulam@ufl.edu',
        url='https://www.python.org/',
        package_dir = {'': 'src'},
        packages=['xmega_driver'],
     )