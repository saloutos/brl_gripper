# brl_gripper

## Purpose of this repo

This codebase lives between controller code and simulation and hardware domains.
It handles setup and communication for the hardware, the simulation, and the renderer.
The goal of this structure is to allow for identical controller code to be deployed in simulation and on hardware,  with as small of a sim-to-real gap as possible.

## Installation instructions

Clone repository.

Optional: activate virtual environment.

Run "pip install ." in top-level directory of the repo.

After pulling any changes to this repo, re-installation will be necessary.

# Random Notes

## Passive Viewer ##

On Linux (Ubuntu 20.04), need to comment out line 463 in the lanuch_passive() function declaration in mujoco.viewer.py: "thread.daemon = True" to avoid some GLX errors and segfaults.

Will need to re-comment this if the MuJoCo package is updated.

May still get a warning about NV-GLX, but at least it isn't an error.

Because of this, entering CTRL+C in the terminal doesn't kill program properly. Need to close viewer instead to avoid segfaults.

## Custom viewer on MacOS ##

Copying the mujoco.viewer file to a local directory and calling launch_passive() results in an error, even with the mjpython executable.

## Simulation timing

Running a dummy controller with SIM_ONLY mode takes approximately 0.2ms per sim loop with a control dt of 2ms, so sim with platform class but no controller is about 10x faster than real time.