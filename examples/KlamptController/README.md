A set of examples and tests on how to integrate SoT and a generic simulator, in this case, Klampt.

`KlamptController` (`src/KlamptController.cpp`) is a class that allows to get and set the status from the internal model used by the controller. It is provided as a c++ library and as python bindings (`KlamptController.i`).

By extending `KlamptController`, `ExampleKlamptController` (`example_klampt_controller.*`) creates a stack.
By calling `dq = exampleKlamptController.computeControl(q)` the `ExampleKlamptController` updates the internal model, udpated the stack, and solves the IK problem to obtain the desired joint velocity to command to the robot.

`klampt_huboplus_controller.py` is a controller example for Klampt [standardized Python control API](http://motion.pratt.duke.edu/klampt/tutorial_custom_controller.html), a.k.a. `controller.py` which uses the Python bindings for `ExampleKlamptController`.

The python scripts `example_client_*` will connect to the stack created by `ExampleKlamptController` and send reference commands. `example_trajectories.py` is a simple library that is used by `example_client_trajectories.py` to create smooth references for the tasks.