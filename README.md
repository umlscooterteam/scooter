---
# Scooter 2.0 - ReadMe
[Latest Version of Docs](https://scooter.readthedocs.io/)

--- 
## Driver installation
[Manipulation driver installation instructions](scooter_manipulation/README.md)

---
## Running
```
ros2 launch scooter_core scooter_core.launch.py
```
---
## Documentation Setup

### Requirements
```shell
$ pip3 install -r docs/requirements.txt 
$ sudo apt install doxygen doxygen-gui
```

### Building Docs
```shell
$ cd scooter/docs/source/doxygen
$ doxygen
$ cd ../..
$ make clean # If needed
$ make html
```


### Updating Docs (Python Added)
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
 
### Updating Docs (C++ Added)
```shell
$ cd scooter/docs/source/doxygen
$ doxywizard # Configure Doxyfile as needed for new files
$ cd ../modules 
$ vim modules.rst
# Create a new RST file for a C++ module by hand using the commands linked below
```

* [.rst include directives](https://breathe.readthedocs.io/en/latest/directives.html)
* [Doxygen Comment Style](https://breathe.readthedocs.io/en/latest/directives.html)
* [Doxygen Special Commands](https://www.doxygen.nl/manual/commands.html)

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
