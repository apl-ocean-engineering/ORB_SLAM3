Code and configurations for processing the three `2025_05_23_aquarium_test/` ROS2 bags of data.

This directory uses the `rosbag_convert_euroc.py` script to convert the contents of the ROS2 bags to a EuRoC-like format of left and right images, then uses the "standard" ORBSLAM3 EuRoC examples (e.g. `mono_euroc`, `stereo_euroc`) to process the data.

For organization, I've tried bundling all of the tasks in to a ['Taskfile'](Taskfile.yml), to use it, install [`task`](https://taskfile.dev)

General steps are as follows:

1. Build ORBSLAM and the EuRoC binaries, go to `ORB_SLAM3/` and:

```
./install_apt_dependencies.sh
./build.sh
```

This will build `Release` binaries, which will appear in (for example), the `Examples/EuRoC/Release/` directory.  To build `Debug` or `RelWithDebInfo` binaries:

```
BUILD_TYPE=Debug ./build.sh
```

2. Return to this directory.  Update the [Taskfile](Taskfile.yml):

  * Update `BAGFILES_PATH` to point to the original ROS2 bags if you need to generate the Aquarium-as-EuRoC data sets
  * Set `BUILT_TYPE` to the binaries to use.  This should agree with the 'build.sh` above
  * Set `CONFIG_FILE` to either `aquarium_resize.yaml` (for downsampling by a factor of two) or `aquarium.yaml` (for processing images at full size)

3. Do the following, once:

```
task convert
```

Will generate the EuRoC datasets in the local directory from the Aquarium bagfiles.

4. Run ORBSLAM3:

```
task stereo:rosbag2_2025_05_23-09_29_11
```
