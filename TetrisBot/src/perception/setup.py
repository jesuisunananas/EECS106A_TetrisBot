from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ee106a-tah',
    maintainer_email='danielmunicio360@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ar_tag_identify = perception.ar_tag_identify:main',
        ],
    },
)
