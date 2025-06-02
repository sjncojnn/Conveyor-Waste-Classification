from setuptools import setup, find_packages

package_name = 'conveyor_controller'

setup(
    name=package_name,
    version='0.0.1',
    # Automatically include conveyor_controller and its subpackages (like scripts)
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Controller for conveyor belt robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = conveyor_controller.controller:main',
            'file_spawner = conveyor_controller.scripts.file_spawner:main',
        ],
    },
)
