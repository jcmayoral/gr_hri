# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "CMakeFiles/common_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/common_autogen.dir/ParseCache.txt"
  "CMakeFiles/common_gui_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/common_gui_autogen.dir/ParseCache.txt"
  "CMakeFiles/offline_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/offline_autogen.dir/ParseCache.txt"
  "CMakeFiles/offline_gui_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/offline_gui_autogen.dir/ParseCache.txt"
  "CMakeFiles/online_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/online_autogen.dir/ParseCache.txt"
  "CMakeFiles/online_gui_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/online_gui_autogen.dir/ParseCache.txt"
  "common_autogen"
  "common_gui_autogen"
  "offline_autogen"
  "offline_gui_autogen"
  "online_autogen"
  "online_gui_autogen"
  )
endif()
