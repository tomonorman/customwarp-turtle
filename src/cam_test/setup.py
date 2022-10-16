from setuptools import setup
import os
from glob import glob

package_name = 'cam_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azazdeaz',
    maintainer_email='azazdeaz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_record = cam_test.save_record:main',
            'stop = cam_test.stop:main',
            'nav = cam_test.nav:main'
        ],
    },
)
