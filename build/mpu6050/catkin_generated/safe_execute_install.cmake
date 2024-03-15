execute_process(COMMAND "/home/shihab/omobot_js/build/mpu6050/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/shihab/omobot_js/build/mpu6050/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
