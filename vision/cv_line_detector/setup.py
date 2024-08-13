from setuptools import find_packages, setup

package_name = 'cv_line_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryansh Sharma',
    maintainer_email='Suryansh.Sharma@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cv_line_detector = cv_line_detector.cv_line_detector:main",
        ],
    },
)
