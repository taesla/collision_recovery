# 🤖 Doosan Robot Collision Recovery System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)


두산 로봇(M0609)의 충돌 감지 시 자동 복구를 수행하는 ROS2 기반 시스템입니다.

---

## 📖 Interactive Documentation

GitHub Pages를 통해 시각적인 문서를 제공합니다:

🔗 **[https://taesla.github.io/collision_recovery/](https://taesla.github.io/collision_recovery/)**

| 문서 | 설명 |
|------|------|
| 🔍 크래시 원인 분석 | DSR 드라이버 크래시의 근본 원인 분석 |
| 🎬 크래시 시나리오 | 4컷 만화 형식의 시각적 크래시 시나리오 |
| 📊 시스템 개요 | 전체 아키텍처 및 로봇 상태 다이어그램 |

---

## ⚠️ 문제 상황

충돌 발생 시 로봇이 `SAFE_STOP(5)` 상태로 전환되는데, 이 상태에서 이동 명령(`movel`, `movej`)을 계속 보내면:

```
SAFE_STOP 상태 → 이동 명령 수신 → 상태 검증 없음 → DRCF 버퍼 오버플로우 → 드라이버 크래시
```

## 💡 해결책

**6단계 충돌 복구 시퀀스**로 안전하게 로봇을 복원합니다:

```
1. SAFE_STOP Reset    → 안전 정지 해제
2. Enter RECOVERY(8)  → 복구 모드 진입
3. Jog Z+ 50mm        → 충돌 지점에서 이탈
4. Complete Recovery  → 복구 완료 신호
5. Exit Recovery      → 복구 모드 종료
6. Servo ON           → 정상 운전 재개
```

---

## 🚀 Quick Start

```bash
# Clone
git clone https://github.com/taesla/collision_recovery.git
cd collision_recovery

# Run
python main.py
```

---

## 📁 프로젝트 구조

```
collision_recovery/
├── docs/                    # 📚 GitHub Pages 문서
│   ├── index.html          # 메인 랜딩 페이지
│   ├── crash_analysis.html # 크래시 원인 분석
│   ├── crash_scenario.html # 4컷 시나리오
│   └── system_overview.html# 시스템 아키텍처
├── main.py                  # 메인 진입점
├── recovery.py              # 복구 로직
├── robot_state.py           # 상태 모니터링
├── motion.py                # 모션 제어
├── config.py                # 설정
└── README.md
```

---

## 🔧 Robot States

| 상태 | 코드 | 설명 |
|------|------|------|
| STANDBY | 1 | 대기 상태 |
| MOVING | 2 | 이동 중 |
| SAFE_OFF | 3 | 전원 차단 |
| SAFE_STOP | 5 | 충돌 정지 ⚠️ |
| RECOVERY | 8 | 복구 모드 |

---

## 🔗 관련 프로젝트

- [doosan_rokey_collabo1](https://github.com/taesla/doosan_rokey_collabo1) - 메인 통합 프로젝트
- [doosan-robot2](https://github.com/doosan-robotics/doosan-robot2) - 두산 로봇 ROS2 드라이버

---

## 📄 License

MIT License © 2025
