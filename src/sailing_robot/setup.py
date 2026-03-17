from setuptools import setup

package_name = 'sailing_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophia',
    maintainer_email='sophia@todo.todo',
    description='The sailing_robot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helming = sailing_robot.scripts.helming:main',
            'tasks = sailing_robot.scripts.tasks:main',
            'tack = sailing_robot.scripts.tack:main',
        ],
    },
)

