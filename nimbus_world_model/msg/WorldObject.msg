geometry_msgs/Pose pose     # Object pose
string name                 # Object name
float64 confidence          # Confidence value
bool inGripper              # Flag for whether an object is in the gripper
string onObject             # The name of the object that this object is on (e.g. "table")
string inObject             # The name of the object that this object is in ("" if it's not in anything)
bool isStatic               # Flag for whether an object can't be moved
time lastUpdated            # Time that the object was last detected