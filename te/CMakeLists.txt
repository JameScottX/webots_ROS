cmake_minimum_required(VERSION 3.0.2)
project(te)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  cv_bridge
  image_transport
)


set(CPP_PATH__ controllers/mob)
set(CPP_PATH_OTHER__ src)




add_message_files(
    FILES
    BoolStamped.msg
    Float64Stamped.msg
    Int32Stamped.msg
    Int8Stamped.msg
    RadarTarget.msg
    RecognitionObject.msg
    StringStamped.msg
  )

## Generate services in the 'srv' folder
  add_service_files(
    FILES
    #camera_get_focus_info.srv
    #camera_get_info.srv
    #camera_get_zoom_info.srv
    #display_draw_line.srv
    #display_draw_oval.srv
    #display_draw_pixel.srv
    #display_draw_polygon.srv
    #display_draw_rectangle.srv
    #display_draw_text.srv
    #display_get_info.srv
    #display_image_copy.srv
    #display_image_delete.srv
    #display_image_load.srv
    #display_image_new.srv
    #display_image_paste.srv
    #display_image_save.srv
    #display_set_font.srv
    #field_get_bool.srv
    #field_get_color.srv
    #field_get_count.srv
    #field_get_float.srv
    #field_get_int32.srv
    #field_get_node.srv
    #field_get_rotation.srv
    #field_get_string.srv
    #field_get_type.srv
    #field_get_type_name.srv
    #field_get_vec2f.srv
    #field_get_vec3f.srv
    #field_import_node.srv
    #field_import_node_from_string.srv
    #field_remove_node.srv
    #field_remove.srv
    #field_set_bool.srv
    #field_set_color.srv
    #field_set_float.srv
    #field_set_int32.srv
    #field_set_rotation.srv
    #field_set_string.srv
    #field_set_vec2f.srv
    #field_set_vec3f.srv
    #get_bool.srv
    #get_float_array.srv
    #get_float.srv
    #get_int.srv
    #get_string.srv
    #get_uint64.srv
    #get_urdf.srv
    #gps_decimal_degrees_to_degrees_minutes_seconds.srv
    #lidar_get_frequency_info.srv
    #lidar_get_info.srv
    #lidar_get_layer_point_cloud.srv
    #lidar_get_layer_range_image.srv
    #motor_set_control_pid.srv
    #mouse_get_state.srv
    #node_add_force_or_torque.srv
    #node_add_force_with_offset.srv
    #node_get_center_of_mass.srv
    #node_get_contact_point.srv
    #node_get_field.srv
    #node_get_id.srv
    #node_get_number_of_contact_points.srv
    #node_get_name.srv
    #node_get_orientation.srv
    #node_get_parent_node.srv
    #node_get_position.srv
    #node_get_static_balance.srv
    #node_get_status.srv
    #node_get_type.srv
    #node_get_velocity.srv
    #node_remove.srv
    #node_reset_functions.srv
    #node_move_viewpoint.srv
    #node_set_visibility.srv
    #node_set_velocity.srv
    #pen_set_ink_color.srv
    #range_finder_get_info.srv
    #receiver_get_emitter_direction.srv
    #robot_get_device_list.srv
    #robot_set_mode.srv
    #robot_wait_for_user_input_event.srv
    #save_image.srv
    #set_bool.srv
    #set_float.srv
    #set_float_array.srv
    #set_int.srv
    #set_string.srv
    #skin_get_bone_name.srv
    #skin_get_bone_orientation.srv
    #skin_get_bone_position.srv
    #skin_set_bone_orientation.srv
    #skin_set_bone_position.srv
    #speaker_is_sound_playing.srv
    #speaker_speak.srv
    #speaker_play_sound.srv
    #supervisor_get_from_def.srv
    #supervisor_get_from_id.srv
    #supervisor_movie_start_recording.srv
    #supervisor_set_label.srv
    #supervisor_virtual_reality_headset_get_orientation.srv
    #supervisor_virtual_reality_headset_get_position.srv
  )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
) 


# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES te
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)








# ORB Part **************************************** 

set(ORB_PATH__  /home/jame/Public/ORB_SLAM3)

LIST(APPEND CMAKE_MODULE_PATH ${ORB_PATH__}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)


include_directories(
  ${ORB_PATH__}
  ${ORB_PATH__}/include
  ${ORB_PATH__}/include/CameraModels
  ${Pangolin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_PATH__}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_PATH__}/Thirdparty/g2o/lib/libg2o.so
${ORB_PATH__}/lib/libORB_SLAM3.so
-lboost_system
)

#aux_source_directory(${CPP_PATH_OTHER__} orb_source_list)
add_executable(orb3test ${CPP_PATH_OTHER__}/orb3_odom.cpp)
add_executable(orb3_imu ${CPP_PATH_OTHER__}/orb3_imu.cpp)

target_link_libraries(orb3test
${LIBS}
${catkin_LIBRARIES}
)

target_link_libraries(orb3_imu
${LIBS}
${catkin_LIBRARIES}
)

# ORB Part **************************************** 












include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

aux_source_directory(${CPP_PATH__} source_list)
add_executable(mob ${source_list})

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(mob te_generate_messages_cpp)

target_link_libraries(mob
   ${catkin_LIBRARIES}
 )


catkin_install_python(PROGRAMS scripts/webots_launcher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})





# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )


# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


