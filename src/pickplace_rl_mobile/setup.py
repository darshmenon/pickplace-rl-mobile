from setuptools import find_packages, setup

package_name = 'pickplace_rl_mobile'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display_launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Pick and place RL mobile robot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = pickplace_rl_mobile.perception_node:main',
            'manip_rl_node = pickplace_rl_mobile.manip_rl_node:main',
            'safety_guard = pickplace_rl_mobile.safety_guard:main',
        ],
    },
)