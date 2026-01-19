from setuptools import setup, find_packages

package_name = 'pyserial'  # Nama package Anda

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Secara otomatis menemukan semua package dalam direktori ini
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],  # Pastikan pyserial ada di sini
    zip_safe=True,
    maintainer='Bobby',
    maintainer_email='bobby.lumbangaol17@gmail.com',
    description='Package for controlling Arduino via serial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coba1 = pyserial.coba1:main',  # Pastikan 'main' didefinisikan di serialcoba.py
            'motor_control_node = pyserial.motor_control_node:coba1',  # Node kontrol motor Anda
        ],
    },
)
