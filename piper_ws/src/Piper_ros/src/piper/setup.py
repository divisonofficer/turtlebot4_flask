from setuptools import find_packages, setup
import glob
import sys
import os
from glob import glob

package_name = 'piper'

python_version = f'{sys.version_info.major}.{sys.version_info.minor}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Piper robot ROS2 control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 기본 노드들
            'piper_single_ctrl = piper.piper_ctrl_single_node:main',
            'piper_read_slave_joint = piper.piper_read_slave_joint:main',

            # 자동 동작 노드
            'auto_motion = piper.auto_motion:main',

            # 서비스 노드 추가
            'move_arm_service = piper.move_arm_service:main',
            'auto_motion_service = piper.auto_motion_service:main',
        ],
    },
)
