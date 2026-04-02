cmake_minimum_required(VERSION 3.8)
project(ur_sim_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS scripts/gravity_compensation.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
