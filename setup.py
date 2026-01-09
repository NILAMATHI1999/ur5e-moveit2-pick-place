from setuptools import find_packages, setup

package_name = 'hri_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nila',
    maintainer_email='nila@example.com',
    description='ROS2 manipulation utilities (MoveIt2 pick&place, planning scene, etc.)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planar_arm = hri_manipulation.planar_arm:main',
            'ik_node = hri_manipulation.ik_node:main',
            'admittance_node = hri_manipulation.admittance_node:main',
            'jacobian_torque_node = hri_manipulation.jacobian_torque_node:main',
            'pick_place_fsm = hri_manipulation.pick_place_fsm:main',
            'pose_sender = hri_manipulation.moveit_pose_sender:main',
            'add_object = hri_manipulation.add_object:main',
            'attach_object = hri_manipulation.attach_object:main',
            'detach_object = hri_manipulation.detach_object:main',
            'fake_object_pose = hri_manipulation.fake_object_pose_publisher:main',
        ],
    },
)
