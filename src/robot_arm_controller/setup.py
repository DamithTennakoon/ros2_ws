from setuptools import find_packages, setup

package_name = 'robot_arm_controller'

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
    maintainer='rpi',
    maintainer_email='rpi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_coord_controller = robot_arm_controller.keyboard_coord_controller:main",
            "hand_coord_controller = robot_arm_controller.hand_coord_controller:main",
            "variable_gripper_controller = robot_arm_controller.variable_gripper_controller:main",
            "key_ee_ctrl = robot_arm_controller.key_ee_ctrl:main",
            "hand_tracking_ee_ctrl = robot_arm_controller.hand_tracking_ee_ctrl:main"
        ],
    },
)
