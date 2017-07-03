from setuptools import setup, find_packages

setup(
    name='mp_planner2',
    version='0.0.0',
    author='Xiaodi Hou',
    author_email='xiaodi.hou@tusimple.ai',
    description='Basic Planner',
    classifiers=['Private :: Do Not Upload'],
    install_requires=['rospy'
                      ],
    url='https://github.com/TuSimple/mp-planner2',
    packages=find_packages(exclude="tests")
)
