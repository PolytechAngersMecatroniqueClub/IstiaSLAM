WARNING : OLD! pa_slam is the new SLAM vesion!!!


# IstiaSLAM
ROS Lidar only based 2D SLAM

This package is based on the work of Philippe Lucidarme and Sebastien Lagrange, and also on the first ROS implementation done by Vincent Cueille.

## Basic SLAM idea

This is a LiDAR only 2D SLAM (Simultaneous Localization and Mapping) approach. The objective is to estimate the 2D environment using a probability map, i.e. a grid in wich each cell contains a probability score of being an obstacle in the environment.

To do that, two maps are considered: a probability map, to store the environment modelization, and a cost map that stores for each cell the distance to the closest obstacle. The cost map is used to be able to give a score (sum of distances) for a LiDAR scan according to a 2d pose (x, y, theta). This give us a cost function.

Using the cost function, a nelder and mead algorithm is considered to "map" new measurement into the map (https://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method).

## the nodes
work in progress...

## The classes

### IstiaSlam

This class is the main class of the SLAM approach. It has an IstiaSlam_Map variable to handle the probability map and the cost map. It also contains the Nelder and Mead algorithm implementation.

Here are the usefull methods of the class:

````
    // function that add the laser scan in the map considering that the sensor is at a pose pose
    void add_2_map(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose);

    // to publish the probability map
    void publish_probability_map();
    
    // to convert the costmap into a grid map and publish it (debugging purpose)
    void publish_cost_map();

    // evaluate the cost of a laser scan according to the pose pose_init (the cost is based on the cost map)
    double get_cost(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose_init);
    
    // implementation of the wikipedia nelder mead algorithm
    geometry_msgs::Pose2D nelder_mead(const sensor_msgs::LaserScan &scan, const geometry_msgs::Pose2D& pose_init);
````

### IstiaSlam_Map

This class handles the probability map and the cost map. The probability map is saved in a nav_msgs::OccupancyGrid variable. The cost map is based on a custom costmap class presented bellow. 

### IstiaSlam_Costmap

This class allows to handle the cost map.

