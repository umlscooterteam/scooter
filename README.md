---
# Scooter 2.0 - ReadMe
[Latest Version of Docs](https://scooter.readthedocs.io/)

--- 

## Documentation Setup

### Requirements
```shell
$ pip3 install sphinx_rtd_theme
$ pip3 install m2r2
```

### Building Docs
```shell
$ cd scooter/docs
$ make clean # If needed
$ make html
```

### Updating Docs
```shell
$ cd scooter/docs
$ sphinx-apidoc ../scooter_PACKAGECHANGED/scooter_PACKAGECHANGED -o source/modules
$ cd source/modules
$ vim modules.rst
# Add the new .rst files name to the modules.rst file as shown
$ cd ..
$ vim conf.py
# append the syspath for any new module added as shown
```

* Warning about duplicate contents is fine as long as it looks correct
* Warning about document or segment not beginning with a transition is fine as long as it looks correct

### Initial Setup
```shell
$ cd scooter
$ mkdir docs
$ cd docs
$ sphinx-quickstart
$ mkdir source
$ cd source
$ mkdir modules
$ sphinx-apidoc ../../scooter_PACKAGENAME/scooter_PACKAGENAME -o modules
```
* Add required modules to the conf.py file
* Edit conf.py to support .md
* Change theme to desired and set code highlight color
* Rerun apidoc and adjust modules.py by hand when updating
* Warning about duplicate contents is fine as long as it looks correct

---
## deploy.sh

### Usage
```shell
$ ./deploy.sh
```

This script will use `rsync` to copy the entire `scooter` package to the workspace specified in the `REMOTE_WS` variable. Please check this carefully since it will overwrite anything in this directory! This script is intended to deploy a working version of the package to the robot and will overwrite the package on the robot! Be very careful when using this. The `-y` flag will skip the confirmation prompt.

---
