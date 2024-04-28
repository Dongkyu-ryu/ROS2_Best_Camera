from setuptools import find_packages, setup
import os
import glob
package_name = 'best_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jundoo',
    maintainer_email='rdk5607@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = best_camera.img_publisher:main',
            'noise_publisher = best_camera.noise_publisher:main',
            'camera_service_server = best_camera.camera_service_server:main',
            'canny_publisher = best_camera.canny_publisher:main'

        ],
    },
)
