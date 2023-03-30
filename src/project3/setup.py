from setuptools import setup
import glob

package_name = 'project3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carson Duffy',
    maintainer_email='cwduffy01@tamu.edu',
    description='Package for project 3',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "track = project3.track:main",
            "people = project3.people.main"
        ],
    },
)
