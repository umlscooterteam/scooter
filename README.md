# Scooter 2.0 - ReadMe

## Documentation Setup
Initial Setup
```shell
$ pip3 install pytest
$ pip3 install sphinx_rtd_theme
$ pip3 install m2r2

$ cd scooter
$ mkdir docs
$ cd docs
$ sphinx-quickstart
```
* Add required modules to the conf.py file
* Edit conf.py to support .md
* Change theme to desired and set code highlight color

## deploy.sh

### Usage
```shell
$ ./deploy.sh
```

This script will use `rsync` to copy the entire `scooter` package to the workspace specified in the `REMOTE_WS` variable. Please check this carefully since it will overwrite anything in this directory! This script is intended to deploy a working version of the package to the robot and will overwrite the package on the robot! Be very careful when using this. The `-y` flag will skip the confirmation prompt.
