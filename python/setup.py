#!/usr/bin/python3

# a quick first hack/test

from setuptools import setup, Extension

setup(name='auracore',
      version='1.0',
      description='test',
      author='Curtis L. Olson',
      author_email='curtolson@flightgear.org',
      url='https://github.com/AuraUAS',
      ext_modules=[
          Extension('auracore.wgs84',
                    define_macros=[('HAVE_PYBIND11', '1')],
                    sources=['util/wgs84.cxx'],
                    depends=['util/wgs84.hxx']
          ),
          Extension('auracore.windtri',
                    define_macros=[('HAVE_PYBIND11', '1')],
                    sources=['util/windtri.cxx'],
                    depends=['util/windtri.hxx']
          )
      ]
      )
