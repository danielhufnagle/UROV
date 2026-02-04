from setuptools import find_packages, setup

package_name = 'turtlesim_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Hufnagle',
    maintainer_email='danielhufnagle2027@u.northwestern.edu',
    description='Control turtlesim with a joystick',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtlesim_joystick = turtlesim_joystick.turtlesim_joystick_function:main'
        ],
    },
)
