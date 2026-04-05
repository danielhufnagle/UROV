from setuptools import find_packages, setup

package_name = 'joystick_3d'

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
    maintainer='danielhufnagle',
    maintainer_email='danielchufnagle@gmail.com',
    description='3 dimensional twist generation from joystick input',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_3d = joystick_3d.joystick_3d_function:main'
        ],
    },
)
