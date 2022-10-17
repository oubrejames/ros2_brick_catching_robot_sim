from setuptools import setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/show_turtle.launch.py', 'launch/run_turtle.launch.py', 
                                   'launch/turtle_arena.launch.py', 'urdf/turtle.urdf.xacro','urdf/test.urdf', 
                                   'rviz/urdf.rviz', 'config/turtle.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oubre',
    maintainer_email='oubrejames@gmail.com',
    description='A package for ME 495 Hw2. Simulates a turtle catching a brick',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtle_robot = turtle_brick.turtle_robot:main', 'arena = turtle_brick.arena:main',
        ],
    },
)

