# What is this?
This library implements various Iterative Closest Point algorithm approaches. The main well known approaches are already implemented: 

- Point to Point ICP in SE3: Minimises the distance between each point on its nearest neighbour
- Point to Plane ICP in SE3: Minimises the distance between each point on its projection on the plane locally defined at the nearest neighbour (projection along plane normal)
- Point to Point ICP in Sim3: Performs the minimisation in the Sim(3) group, thus allowing to estimate a scale factor between two point clouds
- Point to Plane ICP in Sim3

For all methods, it is possible to use MEstimators to robustely discard outliers. DISCLAIMER: this feature has been poorly tested.

# TODO
- Non uniform scaling estimation
- Constraints

# How to build
- Fist install google logging library
  
  sudo apt-get install libgoogle-glog-dev

Then, install the icp library as follow

  mkdir build
  cd build
  # set EXAMPLES to ON if you want sample binaries,
  # set TEST to ON if you want to build the unit tests
  ccmake ..
  make -j4
  # If you want to compile the doxygen documentation for the project
  make doc 
  # This will install the library and headers to /usr/local
  sudo make install

