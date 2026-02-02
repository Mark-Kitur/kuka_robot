from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'kuka_robot'

def generate_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*')),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/urdf', glob('urdf/*')),
        (f'share/{package_name}/worlds', glob('worlds/*')),
    ]

    # Recursively install meshes while preserving directory structure
    for root, _, files in os.walk('meshes'):
        if files:
            install_path = os.path.join('share', package_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_list))

    # Recursively install models while preserving directory structure
    for root, _, files in os.walk('models'):
        if files:
            install_path = os.path.join('share', package_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_list))

    return data_files


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='kimutai.workspace@gmail.com',
    description='KUKA KR210 robot with meshes, models, URDF, and Gazebo integration.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'control_robot = kuka_robot.control:main',
        ],
    },
)
