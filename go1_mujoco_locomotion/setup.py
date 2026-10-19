from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'go1_mujoco_locomotion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 模型 / 权重 / 参数 / launch：全部随包安装，开箱即用
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@example.com',
    description='Unitree Go1 quadruped locomotion (MuJoCo + RL policy) as a drop-in ROS 2 node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'locomotion_node = go1_mujoco_locomotion.locomotion_node:main',
            'verify_interface = go1_mujoco_locomotion.verify_interface:main',
        ],
    },
)
