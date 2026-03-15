from setuptools import find_packages, setup

package_name = 'z_stage_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hd-nano',
    maintainer_email='hd-nano@localhost',
    description='Publish Z-stage position and base_link->laser_frame TF for pseudo-3D stacking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'z_stage_tf_pub = z_stage_tf.z_stage_tf_pub:main',
        ],
    },
)
