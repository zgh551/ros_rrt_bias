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
![rviz_init
](https://i.loli.net/2020/11/03/7Ag6BOTIxUzJVki.png)
3. using the map_server to load the map file

```bash
$ rosrun map_server map_server src/ros_rrt_bias/maps/map_test1.yaml
```
in `maps`folder ,with tree test `yaml`file.
![rviz_map_load](https://i.loli.net/2020/11/03/GFa4ye2A6ukmxql.png)
## Related function introduction
this project contain some function as follow:
### obstacle map inflate
we use the vehicle footprint convolution with obstacle map to inflate the obstacle map.this algorithm reference the "Fast Collision Checking for Intelligent Vehicle Motion Planning"article.

- the kernel picture as follow:

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/Screenshot%20from%202020-08-27%2015-02-36.png)

- the obstacle map as follows:

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/Screenshot%20from%202020-08-27%2016-21-01.png)



- the convolution of two picture as follows:

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/Screenshot%20from%202020-08-27%2016-22-17.png)

### distance map

the grip map as follow:

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/Screenshot%20from%202020-09-21%2019-23-01.png)

- using the way of  "Fast 2-D Distance Transformations"paper ,calculate the distance map as follow:

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/Screenshot%20from%202020-09-21%2019-22-32.png)

the relative-coordinates threshold set to zero.

