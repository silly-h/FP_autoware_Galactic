# - Config file for the  package
# It defines the following variables
#  rs_driver_INCLUDE_DIRS - include directories for 
#  rs_driver_LIBRARIES    - libraries to link against
#  rs_driver_FOUND        - found flag

if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(Boost_ARCHITECTURE "-x64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4) # 32-bit
    set(Boost_ARCHITECTURE "-x32")
  endif()
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
endif(${ENABLE_TRANSFORM})

set(rs_driver_INCLUDE_DIRS "/home/orin/autoware_universe/autoware/src/sensor_driver/driver/rslidar_sdk/src/rs_driver/src;/usr/local/rslidar_sdk/include")
set(RS_DRIVER_INCLUDE_DIRS "/home/orin/autoware_universe/autoware/src/sensor_driver/driver/rslidar_sdk/src/rs_driver/src;/usr/local/rslidar_sdk/include")

set(rs_driver_LIBRARIES "pthread;pcap")
set(RS_DRIVER_LIBRARIES "pthread;pcap")

set(rs_driver_FOUND true)
set(RS_DRIVER_FOUND true)
