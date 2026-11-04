from setuptools import find_packages, setup

package_name = 'bateria_sim'

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
        'Simulador de bateria Li-ion 2S para robot '
        'de patrullaje industrial en ROS 2 Humble'
    ),
    license='LicenseRef-Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'bateria_sim_node = bateria_sim.bateria_sim_node:main',
        'nodo_supervisor = bateria_sim.nodo_supervisor:main',
    ],
},
)
