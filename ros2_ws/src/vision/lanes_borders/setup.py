from setuptools import find_packages, setup

package_name = 'lanes_borders'

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
    maintainer='tars',
    maintainer_email='sgmartes19@comunidad.unam.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'lanes_borders=lanes_borders.lanes_borders:main',
            'lanes_borders_monitor=lanes_borders.lanes_borders_monitor:main'
        ],
    },
)
