from setuptools import setup, find_packages

setup(
    name='mujoco_ar',
    version='0.2.0',
    description='A connector to receive position and rotation data from a connected application.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='Omar Rayyan',
    author_email='olr7742@nyu.edu',
    url='https://github.com/yourusername/mujocoar',
    license='MIT',
    packages=find_packages(),
    install_requires=[
        'asyncio',
        'websockets',
        'numpy',
        'psutil',
        'opencv-python',
        'mujoco',
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)