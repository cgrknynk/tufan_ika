cmake_minimum_required(VERSION 3.8)
project(tufan_v2_ws)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  models
  DESTINATION share/tufan_v2_ws
)

install(PROGRAMS
  src/cmd_vel_odom_node.py
  src/arduino_motor_kontrol.py
  src/surus_koprusu.py
  src/turret_node.py
  src/tabela_node.py
  src/scan_front_filter.py
  src/teker_joint_yayinlayici.py
  src/acik_dongu_odom.py
  src/konum_birlestirici.py
  src/goal_manager_node.py
  src/goal_manager_watchdog.py
  src/nav2_cmd_vel_gate.py
  src/send_goal.py
  src/scan_matcher_filter.py
  src/fix_qos_bridge.py
  src/gps_hedef_donusturucu.py
  src/tabela_etap_yoneticisi.py
  src/egim_costmap_ayarlayici.py
  src/arka_kamera_node.py
  src/serbest_yon_takipcisi.py
  src/kaba_harita_olusturucu.py
  src/on_bosluk_nokta_atici.py
  src/saglik_bekcisi.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
