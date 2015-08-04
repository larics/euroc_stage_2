package_xml="""<?xml version="1.0"?>
<package format="2">
  <name>{0}</name>
  <version>{1}</version>
  <description>{0}</description>

  <maintainer email="{2}">{3}</maintainer>
  <license>BSD</license>

{4}

</package>
"""

cmakelists="""cmake_minimum_required(VERSION 2.8.3)
project({0})

find_package(catkin_simple REQUIRED)
catkin_simple()

# cs_add_library(${{PROJECT_NAME}} src/lib.cc)
# cs_add_executable(${{PROJECT_NAME}} src/main.cc)
# find_package(Boost REQUIRED COMPONENTS system serialization filesystem)
# target_link_libraries(${{PROJECT_NAME}} ${{Boost_LIBRARIES}})

# add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
# catkin_add_gtest(${{PROJECT_NAME}}_tests test/TestPinholeCamera.cc)
# target_link_libraries(${{PROJECT_NAME}}_tests ${{PROJECT_NAME}})

cs_install()
cs_export()
"""
