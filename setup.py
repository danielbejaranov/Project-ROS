from setuptools import setup
setup(name='daniel_gap',version='1.0.0',packages=['daniel_gap'],
      data_files=[('share/ament_index/resource_index/packages',['resource/daniel_gap']),
                  ('share/daniel_gap',['package.xml'])],
      install_requires=['setuptools','numpy'],
      entry_points={'console_scripts':['gap_node = daniel_gap.ros_node:main']})
