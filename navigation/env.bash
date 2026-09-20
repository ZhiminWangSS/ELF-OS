nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$nav_root/../mapping/ws_livox/mapping_env.bash"
source "$nav_root/../mapping/reloc2_ws/install/local_setup.bash"
source "$nav_root/ws/install/local_setup.bash"
# Foxy defaults to only 10 local DDS participants; this stack needs more.
export CYCLONEDDS_URI="file://$nav_root/config/cyclonedds.xml"

# Small real-time matrix operations must not fan out across CPU cores.
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
