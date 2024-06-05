# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "luminosity_drone: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iluminosity_drone:/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(luminosity_drone_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg" NAME_WE)
add_custom_target(_luminosity_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "luminosity_drone" "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg" ""
)

get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg" NAME_WE)
add_custom_target(_luminosity_drone_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "luminosity_drone" "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/luminosity_drone
)
_generate_msg_cpp(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/luminosity_drone
)

### Generating Services

### Generating Module File
_generate_module_cpp(luminosity_drone
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/luminosity_drone
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(luminosity_drone_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(luminosity_drone_generate_messages luminosity_drone_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_cpp _luminosity_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_cpp _luminosity_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(luminosity_drone_gencpp)
add_dependencies(luminosity_drone_gencpp luminosity_drone_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS luminosity_drone_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/luminosity_drone
)
_generate_msg_eus(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/luminosity_drone
)

### Generating Services

### Generating Module File
_generate_module_eus(luminosity_drone
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/luminosity_drone
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(luminosity_drone_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(luminosity_drone_generate_messages luminosity_drone_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_eus _luminosity_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_eus _luminosity_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(luminosity_drone_geneus)
add_dependencies(luminosity_drone_geneus luminosity_drone_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS luminosity_drone_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/luminosity_drone
)
_generate_msg_lisp(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/luminosity_drone
)

### Generating Services

### Generating Module File
_generate_module_lisp(luminosity_drone
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/luminosity_drone
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(luminosity_drone_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(luminosity_drone_generate_messages luminosity_drone_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_lisp _luminosity_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_lisp _luminosity_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(luminosity_drone_genlisp)
add_dependencies(luminosity_drone_genlisp luminosity_drone_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS luminosity_drone_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/luminosity_drone
)
_generate_msg_nodejs(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/luminosity_drone
)

### Generating Services

### Generating Module File
_generate_module_nodejs(luminosity_drone
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/luminosity_drone
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(luminosity_drone_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(luminosity_drone_generate_messages luminosity_drone_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_nodejs _luminosity_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_nodejs _luminosity_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(luminosity_drone_gennodejs)
add_dependencies(luminosity_drone_gennodejs luminosity_drone_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS luminosity_drone_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/luminosity_drone
)
_generate_msg_py(luminosity_drone
  "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/luminosity_drone
)

### Generating Services

### Generating Module File
_generate_module_py(luminosity_drone
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/luminosity_drone
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(luminosity_drone_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(luminosity_drone_generate_messages luminosity_drone_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Biolocation.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_py _luminosity_drone_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhishek/catkin_ws/src/luminosity_drone/luminosity_drone/msg/Info.msg" NAME_WE)
add_dependencies(luminosity_drone_generate_messages_py _luminosity_drone_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(luminosity_drone_genpy)
add_dependencies(luminosity_drone_genpy luminosity_drone_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS luminosity_drone_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/luminosity_drone)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/luminosity_drone
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/luminosity_drone)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/luminosity_drone
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/luminosity_drone)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/luminosity_drone
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/luminosity_drone)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/luminosity_drone
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/luminosity_drone)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/luminosity_drone\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/luminosity_drone
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
