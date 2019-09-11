============
Contributing
============

This document describes the way this repository is setup and the steps need to start contributing to this project

Get Started!
------------

Ready to contribute? Here's how to set up `kobuki-vision` for
local development.

1. Open terminal.
2. Clone your fork locally::

    $ git clone git@github.com:HvA-Robotics/kobuki-vision.git

3. Create a branch for local development::

This project makes uses of Git-Flow to install git flow use::

    sudo apt-get install git-flow

After this just execute git flow init inside the main repository folder (Default settings).

After this create a new feature branch using git flow feature start <NAME_OF_FEATURE>,.
During development use git add and commit as normal but use git flow feature publish instead of push.


4. When you're done making changes, check that your changes pass style and unit
   tests, including testing other Python versions with tox::

    $ tox

To get tox, just pip install it.

5. Commit your changes and push your branch to GitHub::

6. Submit a pull request through the GitHub website.

.. _Fork: https://github.com/HvA-Robotics/kobuki-vision/fork

Pull Request Guidelines
-----------------------

Before you submit a pull request, check that it meets these guidelines:

1. The pull request should include tests.
2. If the pull request adds functionality, the docs should be updated. Put
   your new functionality into a function with a docstring, and add the
   feature to the list in README.rst.
3. The pull request should work for Python 3 and up , and for PyPy.
   Check https://travis-ci.com/HvA-Robotics/kobuki-vision
   under pull requests for active pull requests or run the ``tox`` command and
   make sure that the tests pass for all supported Python versions.


Tips
----

To run a subset of tests::

	 $ py.test test/test_kobuki-vision.py
