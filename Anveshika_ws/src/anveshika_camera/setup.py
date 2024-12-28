from setuptools import find_packages, setup

package_name = 'anveshika_camera'

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
    maintainer='kb',
    maintainer_email='kb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_reader_node = anveshika_camera.aruco_reader_node:main',
            'camera_streamer = anveshika_camera.camera_streamer:main',
            'viewer_node = anveshika_camera.viewer_node:main',
        ],
    },
)
