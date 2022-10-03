from setuptools import setup

setup(
    name='py_arm',
    version='0.1',
    packages=['py_arm'],
    url='',
    license='',
    author='Nathan Sprague',
    author_email='nathan.r.sprague@gmail.com',
    description='Simple 2D Arm Simulator',
    install_requires=[
        'setuptools>=46.1.3',
        'numpy>=1.13.3',
        'Shapely>=1.7.0',
        'matplotlib>=2.1.1',
    ],
    scripts=['scripts/arm_animation.py']
)
