# Source in every local mapping terminal: driver, verification, and FAST-LIO.
# Keep mapping separate from the robot control DDS domain.
source /opt/ros/foxy/setup.bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

# Foxy/CycloneDDS otherwise searches only ten participant indices. Repeated
# local launch cycles can exhaust that range while discovery leases expire.
cyclone_config="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../navigation/config" && pwd)/cyclonedds.xml"
if [[ -f "$cyclone_config" ]]; then
  export CYCLONEDDS_URI="file://$cyclone_config"
fi
