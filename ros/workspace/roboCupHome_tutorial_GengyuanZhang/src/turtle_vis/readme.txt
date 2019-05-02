README.txt

steps:
1. roslaunch turtle_vis TurtleVis.launch
2. you have two choices to set a desired position
  1) run the node "rosrun turtle_vis turtle_set_position"
    NOTE: when you input your desired position, follow the format "x,y,theta" wi
th COMMA
  2) run the service as "rosservice turtle_vis /TurtlePose x y theta"
~                                                                        
