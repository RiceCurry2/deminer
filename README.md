# deminer
This package introduces a polygon selection tool into rVIZ, this tool creates a vector of goals which enforces to make the turtlebot follow a boustrophedon path in the search of landmines. 


## PROGRESS:

### 2 dec. 2020 (Polygon selection)
<a href="http://www.youtube.com/watch?feature=player_embedded&v=NvEaoHpKGbo
" target="_blank"><img src="http://img.youtube.com/vi/NvEaoHpKGbo/0.jpg" 
alt="ROS: Polygon selection" width="350" height="200" border="10" /></a>

### 6 dec. 2020 (Goal definition)
<a href="http://www.youtube.com/watch?feature=player_embedded&v=31A7WzORXv4
" target="_blank"><img src="http://img.youtube.com/vi/31A7WzORXv4/0.jpg" 
alt="ROS: Goal definition " width="350" height="200" border="10" /></a>

## TODO:

* Splitting the client up into client and server (currently resides as a client only).


## Further implementation:
* Add implementation of metal_detector

* create_mine.cpp has been added (dependent on RestrictAreaPlugin) {This might become the deminer_server.cpp}





## This package is dependent on:

    Basic ros packages:
    turtlebot2
    gmapping
    amcl
    rviz

    Other ros packages:
    soundplay

    Plugins:
    RestrictAreaPlugin (Modified) https://github.com/RiceCurry2/RestrictAreaPlugin.git






