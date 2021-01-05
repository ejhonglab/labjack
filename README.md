
#### Installation

Follow [these instructions](https://labjack.com/support/software/installers/exodriver/mac-and-linux/in-depth-build-instructions)
to setup the LabJack Python library. You may replace the portion downloading the
`LabJackPython` code with downloading the zipped latest stable version [here](https://labjack.com/support/software/examples/ud/labjackpython).

Ensure the `python` you are using in the commands to install the Python library is the same as the `python` that is used by your installed version of ROS.

#### Configuration

See `example_params.yaml` for the available ROS parameters and example values.
You can load these parameters as part of a launch file, or manually with either 
`rosparam load <yaml-file>` or `rosparam set /labjack/<param-name> <value>`.


#### Running

After your desired parameters are already set:
`rosrun labjack labjack`

Launching this node can also be integrated as part of a launch file.

