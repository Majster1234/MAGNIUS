string id
string model_id
string configuration_revision
float64[6] start_positions
bool[6] locked
trajectory_msgs/JointTrajectory trajectory
robot_control_interfaces/MksPlanSample[] mks_samples
bool mks_compatible
string mks_message
# Pierwszy etap: zawsze false; odbiór fizyczny jest odrębną bramką.
bool execution_ready
string execution_message
bool simulation_only
bool[6] simulated_start
float64[6] target_positions
string motion_type
