from setuptools import find_packages, setup

package_name = 'ia_to_ros'

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
    maintainer='josep',
    maintainer_email='123630716+JosepMarinG@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ia_to_ros.publisher_member_function:main',
            'model = ia_to_ros.modelRosTest:main',
            'translator = ia_to_ros.TextToROS:main',
            'model2 = ia_to_ros.model2:main',
            'translator2 = ia_to_ros.Translator2:main',
            'textToImage = ia_to_ros.TextToImage_Segmentation:main',
            'imageToText = ia_to_ros.ImageToText_Segmentation:main',
            'compressedToRaw = ia_to_ros.CompressedToRaw:main',
        ],
    },
)
