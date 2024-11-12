# Holistic Planning Framework Mission Planner

## Solver Prerequisites

### Install Gurobi
Go to [the Gurobi website](https://www.gurobi.com/downloads/) and download the Linux version of the Gurobi optimizer. After downloading the tar-gz file, do:

1. Move into `/opt`, move the tar.gz file there, and unpack it. You will need to replace _version_ with your version number.

`cd /opt`

`mv ~/Downloads/gurobi<version>_linux64.tar.gz .`

`tar xvfz gurobi<version>_linux64.tar.gz`

2. Add the following to ~/.bashrc (replacing _version_ with the appropriate value):

```
export GUROBI_HOME="/opt/<version>/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
```

After adding in the above, you will need to either source your `.bashrc` file again or close your current terminal and open a new one.

3. Go to the [Gurobi license page](https://portal.gurobi.com/iam/licenses/list/) and request a new license. If you do not already have an account, you will need to set up one. Request a "Named-User Academic" license. You should get a pop-up window that confirms you have created a new license and gives you some `grbgetkey ###xyz...` command.

4. Run the `grbgetkey` command in a terminal (be sure that you have completed step 2 first). This command may fail if you are not connected to an academic network (e.g. the Mines Wi-Fi). You may need a VPN to complete this step.
5. Move into your Gurobi install directory and build Gurobi (replacing _version_ with the appropriate value).

`cd /opt/gurobi<version>/linux64/src/build/`

`make`

6. Copy the compiled file to the `lib` folder

```cp libgurobi_c++.a ../../lib/```

Gurobi should now work on your system. Note that you will need to request a new license if you move to a new machine.

#### To Build Gurobi C++ Library
Follow these directions: https://stackoverflow.com/a/48867074

### Get LKH Solver
Go to http://webhotel4.ruc.dk/~keld/research/LKH-3/, download the latest version
Build the executable, then move it to 
`~/bin`

Add the following line to your .bashrc file:

`export PATH="/home/$USER/bin:$PATH"`

## To build and run:
To build the project, perform the following from the root directory

```
mkdir build
cd build
cmake ..
make
```

To run the solver, perform the following from the root directory

```
mkdir test/run
cd test/run
../../build/mission-planner scenario_run.txt 3
```

The solver will list input arguments by running

```
../../build/mission-planner
```
