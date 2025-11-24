# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:dcs103e_controller__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:dcs103e_controller__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:dcs103e_controller__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:dcs103e_controller__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:dcs103e_controller__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:dcs103e_controller__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:dcs103e_controller__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:dcs103e_controller__rosidl_typesupport_cpp;__rosidl_generator_py:dcs103e_controller__rosidl_generator_py")

# populate dcs103e_controller_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "dcs103e_controller::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'dcs103e_controller' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND dcs103e_controller_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
