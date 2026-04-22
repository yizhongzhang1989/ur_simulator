cmake_minimum_required(VERSION 3.8)
project(ur_sim_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY mujoco/ DESTINATION share/${PROJECT_NAME}/mujoco)
install(PROGRAMS scripts/gravity_compensation.py
                 scripts/sim_broadcasters.py
                 scripts/ur_type_loader.py
                 scripts/mjcf_postprocess.py
                 scripts/generate_mujoco_model.sh
        DESTINATION lib/${PROJECT_NAME})
# Also install scripts to share for launch file access
install(PROGRAMS scripts/generate_mujoco_model.sh
        DESTINATION share/${PROJECT_NAME}/scripts)
ament_package()
