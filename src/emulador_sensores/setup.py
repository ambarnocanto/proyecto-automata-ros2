
from setuptools import find_packages, setup

package_name = 'emulador_sensores'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laura',
    maintainer_email='launaranjo@est.ecotec.edu.ec',
    description=(
        'Emulador de sensores laterales y frontal '
        'a partir del LiDAR del TurtleBot3'
    ),
    license='LicenseRef-Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emulador_sensores_node = '
            'emulador_sensores.emulador_sensores_node:main',
        ],
    },
)
