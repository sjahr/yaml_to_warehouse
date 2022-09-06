# yaml_to_warehouse

This ROS 2 package converts Robowflex YAML files that represent benchmark motion planning problems into a ROS MoveIt Warehouse database.

A large number of pre-generated YAML files can be downloaded with [this script](https://github.com/KavrakiLab/motion_bench_maker/blob/main/problems/download.sh). Save this script as `download.sh` and run `chmod u+x download.sh && ./download.sh all`.

## Building

Compile the code like so:

    colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache --event-handlers log-

## Running

Run the resulting program like so:

    ros2 run yaml_to_warehouse yaml_to_warehouse --host /tmp/test.db --directory src/yaml_to_warehouse/data --num 5

## Known Issues

* The program runs, but doesn't terminate.
* The program doesn't seem to respect the `--host` argument and saves the data in a file called `localhost`. This file is actually an SQLite database (run `sqlite3 localhost .schema` to check).
