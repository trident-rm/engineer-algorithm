from setuptools import find_packages, setup
from glob import glob

package_name = 'pose_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/cfgs', glob('cfgs/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy<2',
        'apriltag',
        'scikit-learn',
    ],
    zip_safe=True,
    maintainer='billieguo',
    maintainer_email='baoguo0816@gmail.com',
    description='Pose generation ROS 2 package.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_node=pose_gen.data_src:main',
            'aptag_node=pose_gen.aptag:main',
            'corner_node=pose_gen.corner:main',
            'tf_node=pose_gen.tf:main',
            'marker_reco_node=pose_gen.marker_reco:main',
        ],
    },
)
