from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'diff_drive_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
       ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
(os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
(os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
 ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guangwei',
    maintainer_email='wwuguangwei123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
'talker = diff_drive_pkg.talker:main',
'diff_drive_pid = diff_drive_pkg.diff_drive_pid:main',        ],
    },
)
