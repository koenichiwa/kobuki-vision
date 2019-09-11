#!/usr/bin/env python

import os
import sys

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup


if sys.argv[-1] == 'publish':
    os.system('python setup.py sdist upload')
    sys.exit()

readme = open('README.rst').read()
doclink = """
Documentation
-------------

The full documentation is at http://kobuki-vision.rtfd.org."""
history = open('HISTORY.rst').read().replace('.. :changelog:', '')

setup(
    name='kobuki-vision',
    version='0.1.0',
    description='vVision for kobuki project',
    long_description=readme + '\n\n' + doclink + '\n\n' + history,
    author='Melle Toxopeus',
    author_email='melle.toxopeus@gmail.com',
    url='https://github.com/mellow420/kobuki-vision',
    packages=[
        'kobuki-vision',
    ],
    package_dir={'kobuki-vision': 'kobuki-vision'},
    include_package_data=True,
    install_requires=[
    ],
    license='MIT',
    zip_safe=False,
    keywords='kobuki-vision',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: Implementation :: PyPy',
    ],
)
