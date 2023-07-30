# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "sawyer_velctrlsim: 0 messages, 0 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iintera_core_msgs:/home/nhantastrew/Workspaces/sawyer_ws/src/intera_common/intera_core_msgs/msg;-Iintera_core_msgs:/home/nhantastrew/Workspaces/sawyer_ws/devel/share/intera_core_msgs/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sawyer_velctrlsim_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(sawyer_velctrlsim
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sawyer_velctrlsim
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sawyer_velctrlsim_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sawyer_velctrlsim_generate_messages sawyer_velctrlsim_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(sawyer_velctrlsim_gencpp)
add_dependencies(sawyer_velctrlsim_gencpp sawyer_velctrlsim_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sawyer_velctrlsim_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_eus(sawyer_velctrlsim
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sawyer_velctrlsim
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sawyer_velctrlsim_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sawyer_velctrlsim_generate_messages sawyer_velctrlsim_generate_messages_eus)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(sawyer_velctrlsim_geneus)
add_dependencies(sawyer_velctrlsim_geneus sawyer_velctrlsim_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sawyer_velctrlsim_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(sawyer_velctrlsim
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sawyer_velctrlsim
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sawyer_velctrlsim_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sawyer_velctrlsim_generate_messages sawyer_velctrlsim_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(sawyer_velctrlsim_genlisp)
add_dependencies(sawyer_velctrlsim_genlisp sawyer_velctrlsim_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sawyer_velctrlsim_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_nodejs(sawyer_velctrlsim
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sawyer_velctrlsim
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sawyer_velctrlsim_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sawyer_velctrlsim_generate_messages sawyer_velctrlsim_generate_messages_nodejs)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(sawyer_velctrlsim_gennodejs)
add_dependencies(sawyer_velctrlsim_gennodejs sawyer_velctrlsim_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sawyer_velctrlsim_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(sawyer_velctrlsim
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sawyer_velctrlsim
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sawyer_velctrlsim_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sawyer_velctrlsim_generate_messages sawyer_velctrlsim_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(sawyer_velctrlsim_genpy)
add_dependencies(sawyer_velctrlsim_genpy sawyer_velctrlsim_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sawyer_velctrlsim_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sawyer_velctrlsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sawyer_velctrlsim
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sawyer_velctrlsim_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(sawyer_velctrlsim_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(sawyer_velctrlsim_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET intera_core_msgs_generate_messages_cpp)
  add_dependencies(sawyer_velctrlsim_generate_messages_cpp intera_core_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sawyer_velctrlsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sawyer_velctrlsim
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sawyer_velctrlsim_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(sawyer_velctrlsim_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(sawyer_velctrlsim_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET intera_core_msgs_generate_messages_eus)
  add_dependencies(sawyer_velctrlsim_generate_messages_eus intera_core_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sawyer_velctrlsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sawyer_velctrlsim
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sawyer_velctrlsim_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(sawyer_velctrlsim_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(sawyer_velctrlsim_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET intera_core_msgs_generate_messages_lisp)
  add_dependencies(sawyer_velctrlsim_generate_messages_lisp intera_core_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sawyer_velctrlsim)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sawyer_velctrlsim
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sawyer_velctrlsim_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(sawyer_velctrlsim_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(sawyer_velctrlsim_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET intera_core_msgs_generate_messages_nodejs)
  add_dependencies(sawyer_velctrlsim_generate_messages_nodejs intera_core_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sawyer_velctrlsim)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sawyer_velctrlsim\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sawyer_velctrlsim
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sawyer_velctrlsim_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(sawyer_velctrlsim_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(sawyer_velctrlsim_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET intera_core_msgs_generate_messages_py)
  add_dependencies(sawyer_velctrlsim_generate_messages_py intera_core_msgs_generate_messages_py)
endif()
