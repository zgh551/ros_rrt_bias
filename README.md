# ros_rrt_bias
this reposity introduce a planner for vehicle parking, create the obstacle map using picture convolution algorithm.

## Install dependent lib

- **ROS**
- **OMPL**
- **fftw3**
- **map_server**

this project test on the ubuntu20.04

## Run launch

1. enter into the `catkin_ws`folder and running:
```bash
$ catkin_make
```
2. if make is success, then run `roslaunch`
```bash
$ roslaunch src/ros_rrt_bias/luanch/rrt_base_test.launch
```
after that command, you will see follw window:
<img src="https://i.loli.net/2020/11/03/7Ag6BOTIxUzJVki.png" alt="rviz_init" style="zoom: 50%;" />

3. using the map_server to load the map file

```bash
$ rosrun map_server map_server src/ros_rrt_bias/maps/map_test1.yaml
```
in `maps`folder ,with tree test `yaml`file.

<img src="https://i.loli.net/2020/11/03/GFa4ye2A6ukmxql.png" alt="rviz_map_load" style="zoom:50%;" />

## Related function introduction
this project contain some function as follow:
### obstacle map inflate
we use the vehicle footprint convolution with obstacle map to inflate the obstacle map.this algorithm reference the "Fast Collision Checking for Intelligent Vehicle Motion Planning"article.

- the kernel picture as follow:

<img src="https://i.loli.net/2021/08/30/b1wYoaAmj9yIcq2.png" alt="kernel picture" style="zoom:67%;" />

- the obstacle map as follows:

<img src="https://i.loli.net/2021/08/30/sfemwuWlR4ZTXYO.png" alt="obstacle picture" style="zoom: 67%;" />

- the convolution of two picture as follows:

<img src="https://i.loli.net/2021/08/30/2tIevPBaHUmOpL1.png" alt="convolution result" style="zoom:67%;" />

### distance map

the grip map as follow:

<img src="https://i.loli.net/2021/08/30/MnNhpkfHexiRIsL.png" alt="grid map" style="zoom:67%;" />

- using the way of  "Fast 2-D Distance Transformations"paper ,calculate the distance map as follow:

<img src="https://i.loli.net/2021/08/30/3KjQCh1sqJkLHI2.png" alt="distance map" style="zoom: 67%;" />

the relative-coordinates threshold set to zero.

