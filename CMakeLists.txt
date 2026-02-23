cmake_minimum_required(VERSION 3.5)
project(first_challenge_soejima)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(nav_msgs REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()
# 実行ファイルを作成する
ament_auto_add_executable(first_challenge_node
  src/first_challenge.cpp
  src/first_challenge_node.cpp)
# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()
