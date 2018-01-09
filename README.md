# stimuli
Coordinate Arduino stimulus control with computer controlled data acquistion, in a ROS environment.
 
### Purpose
- Keep configuration files describing your experimental stimuli alongside your data. Together with 
  versions of all code used each of your packages, this should constitute full knowledge of the 
  stimuli delivered during an experiment.
  
  See [this ROS package](https://github.com/tom-f-oconnell/metatools) for a means of storing 
  this version information, along with other metadata.
  
- Keep the code on the Arduino constant, to eliminate the ambiguity of which code is executing, 
  and to avoid unwanted states encountered during programming.
  
TODO fill out this README more. include instructions for using all launch files / uploading code
For now, see references to `stimuli` in README [here](https://github.com/tom-f-oconnell/choice/blob/master/README.md).
  
### Example usage
See launch files in my [choice](https://github.com/tom-f-oconnell/choice) repo, where I use 
this package to deliver odors and shocks to flies in a conditioning setup. See also 
[this](https://github.com/tom-f-oconnell/freewalk) work in progress, for delivering odors in
experiments where we change the long term olfactory experience of free-walking flies.
  
### Validation

To run simpler pulse trains of a given duration, without changing the code on the Arduino. 
This launch file will also look for a `stimulus_parameters.yaml` file in the same path, but
with slightly different parameters.
```
roslaunch stimuli pid.launch seconds:=30
```

Parameters (in `stimulus_parameters.yaml`):
- `olf/odor_pin` is the pin to be pulsed. Should control valve delivering odor.
- `olf/separate_balances`
  - `False` if there is not another valve used to keep flow rate more constant
    by opening when not delivering odor.
- `olf/balance` required if `olf/separate_balances` is `True`. The pin to control the 
  balance valve.
- `olf/balance_normally_flowing`
  - `True` if sending the corresponding Arduino pin high closes the valve. I use this 
     configuration to keep the duty cycle of all valves and control circuitry low. This
     helps prevent overheating.
- `olf/odor_pulse_ms`
- `olf/post_pulse_ms`

Launch file arguments:
- `params`: path to `stimulus_parameters.yaml` file.
  - Default: `stimulus_parameters.yaml`, which is in the directory `roslaunch` is called 
    from, in my setup.
- `seconds`: how many seconds to pulse the odor (and balance) pins for
  - Default: 60
- `port`: port of the Arduino. Same port used to program the device in the Arduino IDE.
  - Default: `/dev/ttyACM0`

To test the valves (I run this before and after each experiment, to make sure no connections came loose. With more robust electronics this might be less of a concern.)
- The number of the pin to be switched will be printed in the terminal you run the launch file from.
- I attach higher pressure air to all valves and check manually for air flow at each pin that should be switched.
```
roslaunch stimuli test_valves.launch
```

Press `<Ctrl>-C` to stop the `test_valves.launch` file.
