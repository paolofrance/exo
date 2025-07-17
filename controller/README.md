# Interaction-based reference generation for Implicit Impedance Control

This project contains the high-level reference generator for ODrive's impedance control.

## Build instructions and dependencies

This project has few external dependencies, such as make, Doxygen and Boost. If necessary, install them with:

    sudo apt install -y build-essentials doxygen doxygen-gui libboost1.65-all-dev

Moreover, it depends on a custom **shared memory** implementation that should be already installed in `/usr/local/include`. This is used to communicate with `ODrive-Master`: it sends to ODrive the values of the setpoint and the gains of the impedance controller, and it reads feedback data from the motors (position, velocity, motor torque).

To build the project, we can use `cmake` and `make`:

    mkdir build && cd build/
    cmake ..
    make -j8

This will create the executable file `high_level` in the main folder (i.e., `./`) -- and not in `./build`.

## Usage

It is best to build the code every time before running it. We can do this with

    cd build
    make -j8 && ../high_level

As the two commands are combined with a logical `and`, if the build fails, the code will *not* be executed. 

After starting the `high_level` controller, we can open another terminal and launch `ODrive-Master`. Make sure that `odrv0.use_shmem` is set to `true` (default) and that the modality for ODrive is set to `IMPEDANCE_MODE`. This can be set either pressing `k` during execution on the ODrive-Master screen, or in the code with

    odrv0.working_mode = IMPEDANCE_MODE;

**WARNING**: Please note that if ODrive starts in impedance mode, the motors may move as soon as the code starts running to reach the commanded setpoints. Always make sure motors can output torque safely.

The program can be stopped by pressing CTRL-C on the keyboard; when the `high_level` is stopped, it will send a `SIGINT` interrupt also to ODrive-Master, stopping the motors. Do not stop ODrive-Master before, otherwise this code will run forever (in this situation, it can be forcefully stopped with `kill $(pgrep -f high_level)` from another terminal).

## SAFETY WARNING

This is beta software under development. As such, it must be used with caution at any time. The control algorithms found here are implemented for `AXIS_0` only. 

**NOTE**: Please do not attempt to use this on the low-back exoskeleton before implementing the dual-axis version.