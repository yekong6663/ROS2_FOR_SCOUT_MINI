#!/usr/bin/env bash

_scout_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_scout_local_root="${_scout_repo_dir}/.local_deps/root"

source /opt/ros/humble/setup.bash

export AMENT_PREFIX_PATH="${_scout_local_root}/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${_scout_local_root}/opt/ros/humble:${_scout_local_root}/usr:${_scout_repo_dir}/third_party/fast_lio2_ws/sdk2_install:${_scout_repo_dir}/third_party/fast_lio2_ws/sophus_install:${CMAKE_PREFIX_PATH:-}"
export CPLUS_INCLUDE_PATH="${_scout_local_root}/usr/include:${CPLUS_INCLUDE_PATH:-}"
export LIBRARY_PATH="${_scout_local_root}/usr/lib/aarch64-linux-gnu:${LIBRARY_PATH:-}"
export LD_LIBRARY_PATH="${_scout_local_root}/opt/ros/humble/lib:${_scout_local_root}/opt/ros/humble/lib/aarch64-linux-gnu:${_scout_local_root}/usr/lib:${_scout_local_root}/usr/lib/aarch64-linux-gnu:${_scout_repo_dir}/third_party/fast_lio2_ws/sdk2_install/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${_scout_local_root}/opt/ros/humble/local/lib/python3.10/dist-packages:${_scout_local_root}/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"

source "${_scout_repo_dir}/third_party/fast_lio2_ws/install/setup.bash"
source "${_scout_repo_dir}/third_party/scout_mini_ws/install/setup.bash"
source "${_scout_repo_dir}/my_party/map_transformation_ws/install/setup.bash"
source "${_scout_repo_dir}/my_party/navigation_ws/install/setup.bash"

unset _scout_local_root
unset _scout_repo_dir
