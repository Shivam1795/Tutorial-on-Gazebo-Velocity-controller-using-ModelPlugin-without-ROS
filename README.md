# Tutorial-on-Gazebo-Velocity-controller-using-ModelPlugin
Implementation of Gazebo velocity control plugin (using PID controller) for revolute and prismatic joints of the model using Gazebo transport mechanism instead of ROS transport mechanism and hence this plugin doesn't have any hooks to a robot middleware, like ROS. One of the benefits of using Gazebo with ROS is that it's easy to switch between the real-world and the simulated.

# SDF-model-info
Here, I have used a very simple gazebo model with one revolute and one prismatic joint as shown in gif. Additionally, a visual plugin is used to track trajectories of endpoints of our rotating link in gazebo world.
