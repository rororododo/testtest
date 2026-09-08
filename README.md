# Rocket Reaction Wheel Simulation

로켓 롤(Roll) 자세를 리액션휠과 PID 제어기로 안정화하는 C++ 시뮬레이션입니다.

## 시뮬레이션 내용

| 항목 | 내용 |
|------|------|
| 비행 궤도 | 0 → 10,000m → 0 포물선 (약 60초) |
| 롤 외란 | 랜덤 바람 외란 발생 |
| 제어 방식 | PID 제어기 → 리액션휠 구동 |
| 시각화 | raylib 3D 렌더링 |

## 화면 구성

- **왼쪽**: 3D 로켓 모델 (롤 회전 실시간 표시, 핀이 같이 돌아감)
- **오른쪽**: HUD 패널
  - 고도 / 속도 / 경과 시간
  - 롤 각도 / 롤 각속도 / 바람 외란
  - 리액션휠 RPM / PID 출력값 / 부하 바 그래프

## 키 조작

| 키 | 동작 |
|----|------|
| `SPACE` | 일시정지 / 재개 |
| `R` | 시뮬레이션 리스타트 |
| `ESC` | 종료 |

## 빌드 방법

### 요구 사항
- CMake 3.15+
- g++ (MinGW)
- 인터넷 연결 (첫 빌드 시 raylib 자동 다운로드)

### 빌드 명령

```bash
mkdir build
cd build
cmake .. -G "MinGW Makefiles"
cmake --build .
```

### 실행

```bash
./rocket_sim.exe
```

## 파일 구조

```
.
├── CMakeLists.txt
├── main.cpp                # raylib 시각화 + 시뮬레이션 루프
├── include/
│   ├── rocket.h            # 로켓 상태 및 클래스
│   ├── pid_controller.h    # PID 제어기
│   └── reaction_wheel.h    # 리액션휠 모델
└── src/
    ├── rocket.cpp
    ├── pid_controller.cpp
    └── reaction_wheel.cpp
```
