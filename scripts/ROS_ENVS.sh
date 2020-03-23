##
## Functions which ALL COMPUTERS ON THE ROS NETWORK must call to participate
##

## SUPPORTING FUNCTIONS

# function for allowing odroid-in-the-loop (oil) IP ambiguity resolution 
# for the ground station computer
function unambiguous_hostname() {
    HOSTNAME_I=$(hostname -I)
    if [ ${#HOSTNAME_I} -le 15 ]; then echo "$HOSTNAME_I"
    else echo "192.168.1.1"
    fi
}

# function for setting important ROS variables
function set_ros_vars() {
    export ROS_MASTER_URI="http://$1:11311"
    export ROS_IP=$(unambiguous_hostname)
    export ROS_HOSTNAME=$(unambiguous_hostname)
    env | grep ROS_MASTER_URI
    env | grep ROS_IP
    env | grep ROS_HOSTNAME
}

## ENVIRONMENTS

# For MIT's west high bay motion capture room
function ros_raven() { set_ros_vars 192.168.0.19; }

# west high bay motion capture room with odroid master
function ros_oraven() { set_ros_vars 192.168.0.140; }

# Make the local machine the ROS master
function ros_local() { set_ros_vars localhost; }

# oil environment with GROUND STATION ROS master
function ros_oil_g() { set_ros_vars 192.168.1.1; }

# oil environment with ODROID ROS master
function ros_oil_o() { set_ros_vars 192.168.1.140; }



