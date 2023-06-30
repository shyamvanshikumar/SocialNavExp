from setuptools import setup

package_name = 'socialnav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='corallab-lt',
    maintainer_email='pdhir@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_extracter = socialnav.data_extracter:main',
            'service = socialnav.service_member_function:main',
        ],
    },
)
