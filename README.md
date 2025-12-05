# 🤖 Doosan Robot Collision Recovery

Doosan Robotics 협동로봇(M0609)의 충돌 감지 후 자동 복구 시스템입니다.

## 📋 개요

로봇이 충돌을 감지하면 **SAFE_STOP(5)** 상태로 전환됩니다.  
이 패키지는 6단계 복구 시퀀스를 통해 로봇을 정상 상태로 복귀시킵니다.

## ⚙️ 복구 시퀀스

```
1️⃣ SAFE_STOP 리셋     → set_robot_control(2)
2️⃣ RECOVERY 모드 진입  → set_safety_mode(mode=2, event=2)
3️⃣ Z축 Jog 상승        → jog(Z+, 100mm)
4️⃣ RECOVERY 완료       → set_safety_mode(mode=2, event=0)
5️⃣ RECOVERY 모드 해제  → set_safety_mode(mode=2, event=2)
6️⃣ 서보 ON            → set_robot_control(3)
```

## 📁 파일 구조

```
collision_recovery/
├── main.py          # 진입점 (데모 노드)
├── config.py        # 설정값 (ROBOT_ID, 상태 코드)
├── robot_state.py   # 로봇 상태 조회
├── recovery.py      # 6단계 복구 시퀀스
├── motion.py        # 모션 제어 (Jog, MoveLine)
└── __init__.py      # 패키지 초기화
```

## 🚀 사용법

### 1. 의존성
- ROS2 Humble
- Doosan Robot ROS2 패키지 (`doosan-robot2`)
- `dsr_msgs2` 메시지/서비스 패키지

### 2. 실행

```bash
# ROS2 환경 설정
cd ~/cobot1_ws
source install/setup.bash

# 모듈로 실행
python3 -m collision_recovery.main

# 또는 직접 실행
python3 /path/to/collision_recovery/main.py
```

### 3. 데모 모드

```bash
# 실행 후 메뉴 선택
1. 자동 복구 테스트 (모의 충돌)
2. 현재 상태 확인
3. 수동 복구
0. 종료
```

## 🔧 ROS2 서비스

### 사용되는 서비스

| 서비스 | 타입 | 용도 |
|--------|------|------|
| `/dsr01/system/get_robot_state` | GetRobotState | 상태 조회 |
| `/dsr01/system/set_robot_control` | SetRobotControl | 제어 모드 변경 |
| `/dsr01/system/set_safety_mode` | SetSafetyMode | 안전 모드 설정 |
| `/dsr01/motion/jog` | Jog | Jog 이동 |
| `/dsr01/motion/move_line` | MoveLine | 직선 이동 |
| `/dsr01/aux_control/get_current_posx` | GetCurrentPosx | 현재 위치 조회 |

### 로봇 상태 코드

| 코드 | 상태 | 설명 |
|------|------|------|
| 0 | INITIALIZING | 초기화 중 |
| 1 | STANDBY | 대기 (정상) |
| 2 | MOVING | 이동 중 |
| 3 | SAFE_OFF | 서보 OFF |
| 5 | SAFE_STOP | 충돌 감지 정지 |
| 6 | EMERGENCY_STOP | 비상 정지 |
| 8 | RECOVERY | 복구 모드 |

## 📖 핵심 코드 예시

### 복구 시퀀스 (recovery.py)

```python
def auto_recovery(self) -> bool:
    """6단계 자동 복구"""
    
    # 1. SAFE_STOP 리셋
    self._call_robot_control(2)  # RESET_SAFE_STOP
    
    # 2. RECOVERY 모드 진입
    self._call_safety_mode(mode=2, event=2)
    
    # 3. Z축 상승 (충돌 지점 탈출)
    self._jog_up(distance=100.0)
    
    # 4. RECOVERY 완료
    self._call_safety_mode(mode=2, event=0)
    
    # 5. RECOVERY 모드 해제
    self._call_safety_mode(mode=2, event=2)
    
    # 6. 서보 ON
    self._call_robot_control(3)  # SERVO_ON
    
    return self.state.get_state() == 1  # STANDBY
```

### 상태 확인 (robot_state.py)

```python
def get_state(self) -> int:
    """현재 로봇 상태 코드 반환"""
    request = GetRobotState.Request()
    future = self.cli_state.call_async(request)
    rclpy.spin_until_future_complete(self.node, future)
    return future.result().robot_state
```

## ⚠️ 주의사항

1. **복구 전 주변 확인**: Jog 이동 시 충돌 주의
2. **EMERGENCY_STOP은 별도 처리**: 이 패키지는 SAFE_STOP(5)만 처리
3. **실제 로봇 테스트 시 저속 권장**: 안전 확인 후 속도 조절

## 📚 참고 자료

- [Doosan Robotics Programming Manual V2.12.3](https://www.doosanrobotics.com/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## 📄 라이선스

Apache 2.0

## 👥 Author

Doosan Rokey Collaboration Team
