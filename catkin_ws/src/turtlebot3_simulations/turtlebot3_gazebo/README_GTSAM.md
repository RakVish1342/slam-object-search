### Simple library to build and run GTSAM

The pre-included example binaries will run out of the box most likely. However when including GTSAM in an external project, the following steps will need to be performed and the CMakeLists.txt file as provided in this project can be used. 

**System:** Ubuntu 18.04, GTSAM , Boost-dev 1.65.1, cmake 3.10.2


#### Installation Steps for GTSAM:

1. Install the pre-requirsites:

```
sudo apt-get install libboost-all-dev (boost >= 1.43)
sudo apt-get install cmake (cmake >=3.0)
```

2. Clone the actual GTSAM repo:

```
git clone https://github.com/borglab/gtsam.git (To any folder)
cd gtsam (ie. enter the cloned repo)
```

3. Once inside the repo, follow build instructions as provided:

```
mkdir build
cd build
cmake ..
make check (Optional: long set of conditional checks)
make install (Long installation process. It mainly builds some .so files (and cmake compiler directive files) and adds them to /usr/local/lib.)
```

With this, GTSAM should be installed. To include it in other projects:

1. Create a project folder like this repo
2. Then create a build directory, cmake and make the project.
3. Update ```LD_LIBRARY_PATH``` to also reference ```/usr/local/lib```. This is required since GTSAM dynamically links some libraries like ```/usr/local/lib/libmetis-gtsam.so```. Temporarily do ```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib``` or add that to ```~/.bashrc```


#### Installation with ROS packages

Similar commands from this CMakeLists.txt can be added to the ROS CMakeLists.txt


* Reference1: https://stackoverflow.com/questions/44395859/how-to-use-the-c-library-gtsam-in-my-project-package

* Reference2: https://github.com/raulmur/ORB_SLAM2/issues/494

* Reference3: https://bitbucket.org/gtborg/gtsam/issues/360/couldnt-build-examples, https://github.com/dongjing3309/gtsam-examples
