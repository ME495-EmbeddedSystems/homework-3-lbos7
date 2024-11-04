from setuptools import find_packages, setup

package_name = 'diff_drive'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/ddrive.world.sdf'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Logan Boswell',
    maintainer_email='loganstuartboswell@gmail.com',
    description='TODO: Package description',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
