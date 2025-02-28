cmake_minimum_required(VERSION 3.0.2)
project(drone_demo)

## 指定がなければC++14などにしてもOK (ROS NoeticではC++14推奨)
# add_compile_options(-std=c++11)

## 必要なパッケージを指定
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # ヘッダファイルを含む場合はINCLUDE_DIRSへ追記
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    mavros_msgs
)

###########
## Build ##
###########

## ヘッダファイルのあるディレクトリをインクルードパスに追加
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 実行ファイルの生成
add_executable(multi_drone_controller
  src/multi_drone_controller.cpp
)

## 実行ファイルとcatkinライブラリのリンク
target_link_libraries(multi_drone_controller
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## 実行ファイルをインストール（必要に応じて）
install(TARGETS multi_drone_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## ヘッダファイルをインストール（必要に応じて）
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
