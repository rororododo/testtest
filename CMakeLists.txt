cmake_minimum_required(VERSION 3.15)
project(rocket_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 소스 파일
set(SOURCES
    main.cpp
    src/rocket.cpp
    src/pid_controller.cpp
    src/reaction_wheel.cpp
)

# 실행 파일 생성 (외부 라이브러리 없음)
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE include)

# Windows 링크 라이브러리
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE winmm)
endif()
