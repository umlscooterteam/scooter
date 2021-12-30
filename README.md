# Scooter 2.0

## deploy.sh

### Usage
```
$ ./deploy.sh
```

This script will use `rsync` to copy the entire `scooter` package to the workspace specified in the `REMOTE_WS` variable. Please check this carefully since it will overwrite anything in this directory! This script is intended to deploy a working version of the package to the robot and will overwrite the package on the robot! Be very careful when using this. The `-y` flag will skip the confirmation prompt.
