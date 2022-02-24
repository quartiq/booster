from setuptools import setup, find_packages

setup(name="booster",
      packages=find_packages(),
      version="0.1",
      description="BoosterUtilities",
      author="QUARTIQ GmbH",
      license="Proprietary QUARTIQ",
      install_requires=[
            "miniconf-mqtt@git+https://github.com/quartiq/miniconf@develop#subdirectory=py/miniconf-mqtt"
      ])
