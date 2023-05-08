cmake_minimum_required(VERSION 3.0.2)
project(erp42_serial)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## 메시지 선언(토픽)  #현제 패키지 안에 있는 메시지를 선언한다
add_message_files(FILES ESerial.msg Evelocity.msg setState.msg)


# 의존하는 메시지를 설정하는 옵션
# std_msgs를 설정해야 오류가 안난다
generate_messages(DEPENDENCIES std_msgs)


# 캐킨 패키지 옵션으로 라이브러리, 캐킨 빌드 의존성, 시스템 의존 패키지를 기술함
catkin_package(LIBRARIES ERP42_Serial CATKIN_DEPENDS std_msgs roscpp rospy) #라이브러리, (패키지 파일 이름), 캐킨 디펜드, std, roscpp, rospy


#인클루드 디렉터리 설정
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


