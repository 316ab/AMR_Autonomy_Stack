from setuptools import setup

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haggag',
    maintainer_email='haggag@todo.todo',
    description='Serial bridge for Arduino',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = serial_bridge.serial_bridge:main',
        ],
    },
)
