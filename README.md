**Simple Server Package**

The simple_server package provides a straightforward action server implementation for handling navigation goals. 
It is primarily designed to work with an action client, the GoalPublisher node (from the order_server package) to simulate robot navigation in a cafe or similar type environment.

**Nodes Overview**

**SimpleNavigationServer**

The SimpleNavigationServer node is an action server implementing the NavigateToPose action interface. It receives navigation goals, simulates navigation behavior, and provides feedback during the goal execution.

**Work Flow**

Waits for a navigation goal request from an action client.

Simulates the navigation process, publishing feedback at regular intervals.

Randomly determines if the goal is successfully reached (success rate is sate by 80%).

Responds with the goal result (succeeded or failed).

**command**

  ros2 run simple_server simple_nav_server

**Logs**

    [INFO] [1735352595.596488007] [simple_navigation_server]: Simple Navigation Action Server has been started
    [INFO] [1735352637.952169385] [simple_navigation_server]: Received goal request...
    [INFO] [1735352640.958573151] [simple_navigation_server]: Goal_Reached
    [INFO] [1735352646.565088776] [simple_navigation_server]: Received goal request...
    [INFO] [1735352649.570756266] [simple_navigation_server]: Goal_Reached
    [INFO] [1735352659.589387109] [simple_navigation_server]: Received goal request...
    [INFO] [1735352662.595426159] [simple_navigation_server]: Goal_Failed
    [INFO] [1735352664.607787618] [simple_navigation_server]: Received goal request...
    [INFO] [1735352667.614190671] [simple_navigation_server]: Goal_Failed
    [INFO] [1735352669.625467599] [simple_navigation_server]: Received goal request...
    [INFO] [1735352672.631559993] [simple_navigation_server]: Goal_Reached
    [INFO] [1735352680.389136143] [simple_navigation_server]: Received goal request...
    [INFO] [1735352683.395340463] [simple_navigation_server]: Goal_Reached
    [INFO] [1735352695.649316673] [simple_navigation_server]: Received goal request...
    [INFO] [1735352698.655523814] [simple_navigation_server]: Goal_Reached
    [INFO] [1735352698.663696654] [simple_navigation_server]: Received goal request...
    [INFO] [1735352701.669837503] [simple_navigation_server]: Goal_Reached
