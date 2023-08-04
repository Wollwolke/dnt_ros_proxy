# dtn_robot

This package contains the following nodes used in a simulation environment:

- `control_interface`: Converts Action interface of the `Nav2` WaypointFollower Plugin to a service interface
- `dht_fake_sensor`: Publishes "simulated" temperature and humidity values
- `image_publisher`: Publishes image messages
- `other_sensors`: Publishes "simulated" radiation and pressure values
- `status_publisher`: Publishes real position and "simulated" battery and diagnostic messages

The package is requires the `dtn_robot_interfaces` packge for the custom service definition.
