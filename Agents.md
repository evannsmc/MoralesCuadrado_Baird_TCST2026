1. Keep the blimp stuff in src/ the way it is but change the quad stuff to be submodules of git@github.com:evannsmc/newton_raphson_px4.git, and git@github.com:evannsmc/nmpc_acados_px4.git
2. Note that people may need my mocap packages: git@github.com:evannsmc/vicon4px4.git or https://github.com/evannsmc/optitrack4px4, as well as my other related packages for ROS2/mocap integration: git clone git@github.com:evannsmc/optitrack4px4.git
git clone -b v1.16_minimal_msgs git@github.com:evannsmc/px4_msgs.git
git clone git@github.com:evannsmc/mocap_msgs.git
git clone git@github.com:evannsmc/mocap_px4_relays.git
cd ..   # back to workspace root in the readme of the mocap packages
3. Update the readme accordingly as well with instructions for all users in running the blimp MPC, FBL, NR, as well as the quad NMPC, and NR controllers for users of this codebase that accompanies my paper