from setuptools import find_packages, setup

package_name = 'final_package'

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
    maintainer='jordan',
    maintainer_email='jordan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "py_maze_node = final_package.maze_runner:main",
        "py_simmaze_node = final_package.simmaze_runner:main"
		],
    },
)
