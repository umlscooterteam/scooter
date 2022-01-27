# scooter_manipulation

This package is responsible for providing manipulation for the scooter. It implements the `Pick` service as well as
adding the static collision environment. It also is responsible for launching and configuring MoveIt and the UR5 drivers
to provide motion planning and motion controls.

To bring up everything in this package including the UR5 driver, MoveIt, and the static collision environment, run:
```
$ ros2 launch scooter_manipulation scooter_manipulation.launch.py
```

## static_collision_object_publisher
```static_collision_object_publisher``` is responsible for adding the static collision environment (sensors, scooter
frame, etc.) to the move group's planning scene. Objects are specified as box primitives, which are described in a CSV
file in the following format:

```
name   pos_x   pos_y   pos_z   euler_x   euler_y   euler_z   dim_x   dim_y   dim_z
```

See `ur5_environment.csv` for an example environment.

The `env_csv` parameter allows you to specify a path to a `.csv` file to be parsed by
`static_collision_object_publisher.` To specify a path to a `.csv` file in the `config` directory you might specify the
path as follows:

```python
env_csv_path = PathJoinSubstitution([FindPackageShare("scooter_manipulation"), "config", "ur5_environment.csv"])
```

This node also requires the robot description (URDF) and semantic robot description (SRDF) in order to interact with the
planning scene, which must be passed as parameters. See `ur_driver_bringup.launch` for an example of this.