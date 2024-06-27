from setuptools import setup, find_packages

setup(name='brl_gripper',
      version='0.0.1',
      description='',
      url='https://github.com/saloutos/brl_gripper',
      packages=find_packages(),
      install_requires=['numpy', 'mujoco', 'python-can', 'pyyaml'],
      zip_safe=False,
      include_package_data=True)