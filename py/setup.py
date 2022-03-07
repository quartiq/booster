from setuptools import setup, find_packages

setup(name="booster",
      packages=find_packages(),
      version="0.1",
      description="Booster Utilities",
      author="QUARTIQ GmbH",
      license="Proprietary QUARTIQ Software",
      install_requires=[
            "miniconf-mqtt@git+https://github.com/quartiq/miniconf@develop#subdirectory=py/miniconf-mqtt"
      ])
