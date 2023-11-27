from setuptools import find_packages, setup

package_name = 'uol_cmp9767_tutorial'

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
    maintainer='pulver',
    maintainer_email='pulver22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_listener = uol_cmp9767_tutorial.tf_listener:main',
            'mover = uol_cmp9767_tutorial.mover:main',
            'mover_with_pose = uol_cmp9767_tutorial.mover_with_pose:main',
            'opencv_test = uol_cmp9767_tutorial.opencv_test:main',
            'image_projection_1 = uol_cmp9767_tutorial.image_projection_1:main',
            'image_projection_2 = uol_cmp9767_tutorial.image_projection_2:main',
            'image_projection_3 = uol_cmp9767_tutorial.image_projection_3:main',
        ],
    },
)
