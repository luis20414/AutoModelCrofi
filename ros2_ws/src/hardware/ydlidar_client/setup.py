from setuptools import find_packages, setup

package_name = 'ydlidar_client'

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
            'ydlidar_client = ydlidar_client.ydlidar_client:main',
            'rebase = ydlidar_client.rebase:main',
            'colision = ydlidar_client.colision:main'
        ],
    },
)
