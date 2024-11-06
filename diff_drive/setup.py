from setuptools import find_packages, setup
from pathlib import Path

def recursive_files(prefix, path):
    """
    Recurse over path returning a list of tuples suitable for use with setuptools data_files.
    :param prefix: prefix path to prepend to the path
    :param path: Path to directory to recurse. Path should not have a trailing '/'
    :return: List of tuples. First element of each tuple is destination path, second element is a list of files to copy to that path
    """
    return [(str(Path(prefix)/subdir),
             [str(file) for file in subdir.glob('*') if not file.is_dir()] ) for subdir in Path(path).glob('**')]

package_name = 'diff_drive'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/env-hooks', ['env-hooks/diff_drive.dsv']),
        *recursive_files('share/' + package_name, "models"),
        *recursive_files('share/' + package_name, "worlds"),
        ('share/' + package_name + '/launch', ['launch/ddrive.launch.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Logan Boswell',
    maintainer_email='loganstuartboswell@gmail.com',
    description='A package for simulating a differential drive robot in Gazebo',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
