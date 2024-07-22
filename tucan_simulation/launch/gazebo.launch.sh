#! /bin/sh

# Launch one drone and bridge

get_ros2_package_share_dir() {
    package_name="tucan_simulation"
    
    # Get the install prefix of the package
    package_prefix=$(ros2 pkg prefix "$package_name")
    if [ $? -ne 0 ]; then
        echo "Package '$package_name' not found"
        return 1
    fi
    
    # Construct the share directory path
    share_dir="$package_prefix/share/$package_name"
    
    # Check if the directory exists
    if [ -d "$share_dir" ]; then
        echo "$share_dir"
    else
        echo "Share directory for package '$package_name' not found"
        return 1
    fi
}

share_dir=$(get_ros2_package_share_dir "$package_name")

cd $share_dir
python3 launch/simulation-gazebo.py