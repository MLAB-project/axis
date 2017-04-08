#!/usr/bin/python
# -*- coding: utf8 -*-

from setuptools import setup, find_packages
import sys
import os
import os.path as path


os.chdir(path.realpath(path.dirname(__file__)))


setup(
    name             = 'axis',
    version          = '0.0.1',
    author           = 'Mlab',
    author_email     = 'roman-dvorak@mlab.cz',
    description      = 'HBSTEP axis class',
    long_description = "",
    url              = 'https://github.com/Robozor-network/',
    
    packages    = ['axis'],
    #packages    = find_packages("src"),
    package_dir = {'': 'src'},
    provides    = ['axis'],
    install_requires = [ 'hidapi' ],
    keywords = ['axis', 'hbstep', 'mlab'],
    license     = 'Lesser General Public License v3',
    download_url = 'https://github.com/',
    
    #test_suite = 'axis.tests',
    
    classifiers = [
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
        'Natural Language :: Czech',
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
    ]
)

