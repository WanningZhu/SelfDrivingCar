## Wanning Zhu Project4
This project uses ARA* search algorithm to achieve Path Planning, at the same time all obstacles are driving along their own paths.
#### Command Line
- `catkin_make`.
- `roslaunch an_dynamic_planner scenario2.launch`

#### Explanation
Main file is **planner**([.cpp](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/src/planner.cpp) [.h](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/include/an_dynamic_planner/planner.h)).
##### Load map data
I create **allInfoClass**([.cpp](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/src/allInfoClass.cpp) [.h](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/include/an_dynamic_planner/allInfoClass.h))to store all information about the discrete map, obstacles and Motion Primitives. Map is composed of **mapCell**([.cpp](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/src/mapCell.cpp) [.h](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/include/an_dynamic_planner/mapCell.h)) small rectangular squares(0.74m * 1m), and obstacle is in the center of rectangular square.
Doesn't like Project4, in Project4 the obstacles are moving. So I use a map to store all obstacles' positions at each time step, `map<pair<double time, vector<carState> allObsPositionAtThisTime`.When checking hit at every time step, I just pick the corresponding vector of obstacles position to check if there is a collision now.
##### Check hit
If the distance between the centers of car and obstacle is larger than 2*outer_circle_radius, there is no collision. If it's smaller than 2* inner_circle_radius, there will be a collision. Else we need to further check collision. I use 3 circles to cover the footprint of the car and obstacles. The radius is `sqrt((length/3)^2+(width/2)^2)`, if any two circles on the car and obstacle respectively intersect, there is a collision, otherwise there is no collision.
##### ARA*([.cpp](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/src/pathPlanner.cpp) [.h](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/include/an_dynamic_planner/pathPlanner.h))
Generate the trajectory from the start position to goal position to avoid collisions with all obstacles. Where using **MyMinHeap**([.cpp](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/src/minHeap.cpp) [.h](https://gitlab.com/RubyZhu/student1706133/blob/master/Project4/an_dynamic_planner/include/an_dynamic_planner/minHeap.h))as opened list. Initialize eps=3.0 and decrease eps by 1.0 every ARA* search. Planning from the start cell, generate valid successors cells which car can drive to without collision, and push them into opened list. If this valid successor is in the closed list, push it into Incons. Circulate this process with the top cell in opened list until the top cell's f-value is smaller than goal cell's g-value.