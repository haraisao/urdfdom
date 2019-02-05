include_directories(include)

add_library(urdfdom_world SHARED src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp src/world.cpp)
if(WIN32)
  target_link_libraries(urdfdom_world ${TinyXML2_LIBRARIES} ${console_bridge_LIBRARIES})
else()
  target_link_libraries(urdfdom_world ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES})
endif()
set_target_properties(urdfdom_world PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model SHARED src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp)
if(WIN32)
  target_link_libraries(urdfdom_model ${TinyXML2_LIBRARIES} ${console_bridge_LIBRARIES})
else()
  target_link_libraries(urdfdom_model ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES})
endif()
set_target_properties(urdfdom_model PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_sensor SHARED src/urdf_sensor.cpp)
if(WIN32)
  target_link_libraries(urdfdom_sensor urdfdom_model ${TinyXML2_LIBRARIES} ${console_bridge_LIBRARIES})
else()
  target_link_libraries(urdfdom_sensor urdfdom_model ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES})
endif()
set_target_properties(urdfdom_sensor PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model_state SHARED src/urdf_model_state.cpp src/twist.cpp)
if(WIN32)
  target_link_libraries(urdfdom_model_state ${TinyXML2_LIBRARIES} ${console_bridge_LIBRARIES})
else()
  target_link_libraries(urdfdom_model_state ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES})
endif()
set_target_properties(urdfdom_model_state PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

# --------------------------------

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf urdfdom_model urdfdom_world)

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz urdfdom_model)

# urdf_mem_test is a binary for testing, not a unit test
add_executable(urdf_mem_test test/memtest.cpp)
target_link_libraries(urdf_mem_test urdfdom_model)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

INSTALL(TARGETS urdfdom_model DESTINATION ${CMAKE_INSTALL_LIBDIR})
INSTALL(TARGETS urdfdom_world DESTINATION ${CMAKE_INSTALL_LIBDIR})
INSTALL(TARGETS check_urdf urdf_to_graphiz urdf_mem_test
  DESTINATION ${CMAKE_INSTALL_BINDIR})
INSTALL(TARGETS urdfdom_sensor DESTINATION ${CMAKE_INSTALL_LIBDIR})
INSTALL(TARGETS urdfdom_model_state DESTINATION ${CMAKE_INSTALL_LIBDIR})
INSTALL(DIRECTORY include/urdf_parser DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


if(WIN32)
install(TARGETS urdfdom_model
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  CONFIGURATIONS Release)
install(TARGETS urdfdom_world
   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Release)
install(TARGETS urdfdom_sensor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Release)
install(TARGETS urdfdom_model_state RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Release)
endif()
