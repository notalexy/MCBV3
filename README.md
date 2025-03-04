# MCBV3
Rose-Hulman Robomasters' (ThornBots) controls repository. Uses Taproot. Command-based and started from Teaching-Freshies

## Getting Started

First, **MAKE SURE TAPROOT IS PROPERLY INSTALLED**! If you haven't go ahead and check out our [software portal](https://thornbots-software.web.app) and 
check out our links page to get started with taproot.

Assuming you have taproot, run the following to clone this repository:

```
git clone --recursive git@github.com:Thornbots/MCBV3.git
```

If you use the Docker container, or have already cloned the repository yourself, you should instead
run:

```
git submodule update --init --recursive
```

Finally, install `pipenv` and set up the build tools:

```
pip3 install pipenv
cd MCBV3/MCB-project/
pipenv install
```

Once finished, congratulations! You're ready to start coding!

## Writing and Flashing Code

All code is written in `MCB-Project/src/`. If you understand typical C++
and taproot, this should be relatively intuitive. 

To flash code, you'll need to be in the pipenv shell. If it's not running, use the following command to start it.

```
cd MCBV3/MCB-project/
pipenv shell
```

Note that this shell needs to be running to flash code, so every time you open a terminal you'll need to start it up again. Once it's running, you're ready to flash!
Don't forget, you should be in the `MCBV3/MCB-project` directory when running any of these commands! Once you're ready, run the following:

```
scons run
```

This command will build the robot code, prepping it to flash, then will proceed to flash it using `openocd`. Alternatively, you can run `scons build` to just build the code. You'll notice that this code will default to compiling the standard project. You can specify the robot type and the system identificaton enable using the command below:

```
scons run robot=<ROBOT_TYPE> sysid=<SYSID_TYPE>
```

These arguments will work for `scons build` too!
