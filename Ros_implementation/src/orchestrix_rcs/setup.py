from setuptools import setup

package_name = 'orchestrix_rcs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Mithun',
    maintainer_email='mithun@example.com',
    description='Robot Control System for Orchestrix',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = orchestrix_rcs.robot_node:main'
        ],
    },
)
