from setuptools import find_packages, setup

package_name = 'mission_director'

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
    maintainer='imav2024',
    maintainer_email='mbrummelhuis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mission_director = mission_director.mission_director:main",
            "mission_director_simple = mission_director.mission_director_simple:main",
            "mission_director_AHT = mission_director.mission_director_AHT:main",
        ],
    },
)
