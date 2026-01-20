from setuptools import find_packages, setup

package_name = 'duckie_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ROS package indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # ✅ INSTALL LAUNCH FILES
        ('share/' + package_name + '/launch',
            ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='akhiljithvg444@gmail.com',
    description='Duckie robot bringup (camera + perception + safety + motors)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
