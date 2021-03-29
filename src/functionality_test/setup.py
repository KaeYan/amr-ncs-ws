from setuptools import setup

package_name = 'functionality_test'

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
    maintainer='Kae Yan',
    maintainer_email='kaeyan.goh@ncs.com.sg',
    description='Functionality tests of Step 3 for AMR',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = functionality_test.publisher_member_function:main',
            'test_door_request = functionality_test.test_door_request:main',
            'test_lift_request = functionality_test.test_lift_request:main',
            'test_loop_request = functionality_test.test_loop_request:main',
        ],
    },
)
