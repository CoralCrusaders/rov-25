from setuptools import setup, Extension

module = Extension(
    'nmea_encode',
    sources = ['nmea_encode.c']
)

setup(
    name = 'nmea_encode',
    version = '1.0',
    description = 'Encode gamepad data to NMEA-formatted bytes',
    ext_modules = [module]
)