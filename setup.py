from setuptools import setup

package_name = 'pid_modifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Haoqi Zeng',
    maintainer_email='zeng-hq@sjtu.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plt = pid_modifier.plt_node:main',
            'offer = pid_modifier.offboard_control:main',
            'att = pid_modifier.attitude_node:main'
        ],
    },
)
