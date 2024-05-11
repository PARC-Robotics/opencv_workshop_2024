from setuptools import setup

package_name = 'opencv_workshop_2024'

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
    maintainer='kene-mbanisi',
    maintainer_email='kene.mbanisi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_subscriber = opencv_workshop_2024.image_subscriber:main',
            'find_tomatoes = opencv_workshop_2024.find_tomatoes_ros:main',
        ],
    },
)
