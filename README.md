# Yulbungbung — 회전교차로 동네 심부름 자율주행 (BC → DAgger → PPO)

자율주행 인공지능 및 제어 팀 프로젝트 (팀 율붕붕 / Yulbungbung)

- 프로젝트 페이지: https://venezuela-chocolate-milk.github.io/Yulbungbung/
- 데이터셋 (Hugging Face): https://huggingface.co/datasets/yulbungbung/Yulbungbung-driving-data
- 최종 모델: `models/final_rl/model.pt` (PPO, sha256 `90be09f4626c1e28d0194939a82e9085ef2d8294390ea193ce2a4853ab094431`)

## 팀

| 이름 | 역할 | 세부 역할 | GitHub |
|---|---|---|---|
| 장혁 | SUMO 도로 환경 구현 | 도로 구조 및 차선 구현 | [venezuela-chocolate-milk](https://github.com/venezuela-chocolate-milk) |
| 신준서 | SUMO 도로 환경 구현 | 경로 및 우선순위 규칙 설정 | [SJS-m-e](https://github.com/SJS-m-e) |
| 김태회 | 일반 차량 및 교통 흐름 구성 | 교통량 설정 및 주행 시나리오 구성 | [KIMTAEHOE](https://github.com/KIMTAEHOE) |
| 박정환 | AV 강화학습 모델 구현 | 강화학습 알고리즘 구현 | [righthPark](https://github.com/righthPark) |
| 김리야 | AV 모델 학습 및 데이터 분석 | 시뮬레이션 데이터 분석 및 모델 성능 평가 | [fldilll](https://github.com/fldilll) |

## 1. 문제

SUMO 로 만든 회전교차로가 있는 동네 지도에서, AV 가 약 5.6 km 심부름 경로를 처음부터 끝까지 운전합니다.

- 들를 곳 5곳: 매번 주차장에 들어가 빈 칸 옆에 2초 정차, 마지막 곳 정차 = 완주
- 일반 차량과 만나는 상황: 회전교차로 진입·양보, 신호 교차로 좌회전, 합류, 불법 주정차 구간 추월(차선변경), 긴급차 양보
- 어린이보호구역 (30 km/h, 단속 카메라)
- 안전 마스킹은 끔 (규칙으로 행동을 막아 주지 않음)

## 2. POMDP

| 항목 | 내용 |
|---|---|
| 관측 (163차원) | 기본 자기 정보 11, 신호 5, 차로·화살표 정보, **보이는 차만** (가림 반영) 최대 10대 × 12값 (긴급차 여부 포함), 정차 구역 3 + 가까운 빈 칸 1 + 단속 카메라 1. 강사 전용 정보·다른 차 의도·차선 단위 경로 정보는 넣지 않음 |
| 행동 | 가감속 (연속, Gaussian) + 차선변경 {왼쪽, 유지, 오른쪽} (범주형) |
| 보상 | 진행 +0.1/스텝, 시간 −0.02/스텝, 정차 지점 +10, 완주 +10, 충돌 −50, 막힘·경로 이탈 −50, 적신호 −30 (모두 즉시 종료), 근접·멈춤·과속·차선변경·저크·긴급차 진로 방해 감점 (`env/roundabout_env_config.py` `URBAN["reward"]`) |
| 신경망 | 주변 차 10대 → DeepSets 인코더 (12→64, max+mean), 나머지 43차원과 합쳐 MLP |

## 3. 학습 과정

| 단계 | 방법 | 결과 (완주 % / 충돌 %, 100판) |
|---|---|---|
| ③ 데이터 | 규칙 기반 강사 1000판 (349만 행) + 잡음 넣은 강사(DART) 1000판 (233만 행) | 강사 완주 95.8% |
| ④ BC | 강사 데이터로 모방학습, seed 3개 | 최고 21 / 7 |
| ⑤ DAgger | 모델이 운전하고 강사가 정답을 다는 라운드 반복, 누적 데이터로 다시 학습 | 최고 (3라운드 d3s0) 91 / 2 (500판: 89.6 / 3.6) |
| ⑥ PPO | DAgger 최고에서 출발, PPO + BC 보조 손실 (계수 0.5), 학습률 2.5e-6, 20만 스텝 | **최종 모델** |

강화학습은 설정을 여러 개(A부터 F까지) 시험했고, 모두 20만에서 40만 스텝 사이에서 가장 좋고 그 뒤로 나빠졌습니다. 그래서 짧게 학습하고, 처음 쓰는 seed 로 다시 평가해서 골랐습니다 (운 좋은 체크포인트를 고르지 않으려고).

## 4. 결과 (기준 교통량, 마스킹 끔)

| 모델 | 500판 seed 7000 | 200판 seed 8000 | 100판 seed 5000 |
|---|---|---|---|
| **최종 PPO (E 20만)** | **92.4 / 4.6** | 89.5 / 6.0 | 92 / 3 |
| DAgger 최고 (d3s0) | 89.6 / 3.6 | 89.0 / 7.5 | 87 / 3 |

- 최종 PPO 는 완주가 2.8%p 높고 충돌이 1.0%p 높습니다. 500판 오차(완주 ±2.7, 충돌 ±1.7) 안이라 **DAgger 최고를 확실히 넘었다고 말할 수는 없고, 비슷한 수준**입니다.
- 더 큰 비교(아래 강건성, 같은 5,000판)에서는 최종 PPO 의 완주율이 DAgger 최고보다 1.7%p 높았고, 이 차이는 우연으로 보기 어렵습니다.
- 충돌 5% 기준: 500판 4.6%. 200판 seed 8000 에서는 6.0% 로 경계에 가깝습니다 (같은 200판에서 DAgger 최고는 7.5%).

### 강건성 (처음 보는 seed 5개 × 1000판, 판마다 50% 확률로 긴급차)

| Seed | 판 수 | 완주 | 충돌 | 신호위반 | 막힘·이탈 | 시간초과 |
|---|---|---|---|---|---|---|
| 100,000 | 1,000 | 90.8% | 4.9% | 2.3% | 1.8% | 0.2% |
| 200,000 | 1,000 | 89.6% | 5.2% (≥5%) | 2.7% | 2.1% | 0.4% |
| 300,000 | 1,000 | 90.8% | 5.0% (≥5%) | 2.5% | 1.1% | 0.6% |
| 400,000 | 1,000 | 91.3% | 4.3% | 1.7% | 2.3% | 0.4% |
| 500,000 | 1,000 | 90.9% | 4.3% | 2.0% | 2.2% | 0.6% |
| **합계** | **5,000** | **90.7%** | **4.74%** | 2.2% | 1.9% | 0.4% |

합계 충돌률 4.74% (95% 범위 약 4.2~5.3%). 평균은 5% 미만이지만 기준선에 가깝고, seed 2개는 5.0% 이상이었습니다. seed별 차이는 1,000판 표본 오차(±1.4%p) 범위입니다. 강건성 조건은 일반 평가보다 어렵습니다 (판마다 50% 확률로 긴급차 추가).

#### DAgger 최고와 비교 (같은 seed, 같은 5,000판)

| Seed | 최종 PPO 완주 / 충돌 | DAgger 최고 완주 / 충돌 |
|---|---|---|
| 100,000 | 90.8% / 4.9% | 90.6% / 4.5% |
| 200,000 | 89.6% / 5.2% | 88.9% / 6.1% |
| 300,000 | 90.8% / 5.0% | 90.0% / 4.9% |
| 400,000 | 91.3% / 4.3% | 87.8% / 6.2% |
| 500,000 | 90.9% / 4.3% | 87.6% / 5.5% |
| **합계** | **90.7% / 4.74%** | **89.0% / 5.44%** |

- 판마다 seed 가 고정이라 두 모델이 같은 교통 상황에서 출발합니다. 판끼리 짝지어 보면 PPO 만 완주한 판 450개, DAgger 만 완주한 판 365개입니다. 짝 검정(McNemar) z = 2.98 (p ≈ 0.003) 이라 **완주율은 PPO 가 확실히 높습니다**. 5개 seed 모두 PPO 가 높았습니다.
- 충돌률도 PPO 가 0.7%p 낮지만 (PPO 만 충돌 209판, DAgger 만 충돌 244판, z = −1.64, p ≈ 0.10) 확실한 차이라고 하기는 어렵습니다.
- 완주한 판의 평균 시간: PPO 1,756 s, DAgger 1,817 s (PPO 가 약 3% 빠름).
- 판별 결과 파일 (`RL_s*.json`, `DG_s*.json`) 은 Hugging Face 데이터셋의 `robustness/` 에 있습니다. DAgger 는 `models/dagger_best/` 모델로 같은 명령을 돌렸습니다.

## 5. 실행 방법

Ubuntu 22.04, Python 3.10 기준.

```bash
python3 -m venv ~/sumo-venv
~/sumo-venv/bin/pip install -r requirements-sumo-driving.txt

# 최종 모델 평가 (작업자 수는 CPU 에 맞게)
~/sumo-venv/bin/python eval_parallel.py models/final_rl/model.pt --episodes 100 --workers 8 --seed 5000

# 강건성 평가 (판마다 seed 고정, 긴급차 확률 0.5)
~/sumo-venv/bin/python eval_robust.py models/final_rl/model.pt --episodes 1000 --workers 8 --seed 100000 --ev-prob 0.5

# 주행 영상 (sumo-gui 필요)
~/sumo-venv/bin/python record_video.py --model models/final_rl/model.pt --route mom --fps 20

# 데이터 수집 (제출 데이터와 같은 설정)
~/sumo-venv/bin/python collect_pomdp_data.py --policy ppo --model models/final_rl/model.pt \
  --episodes 700 --workers 8 --seed 1000000 --ev-mode eval --state-radius 60 --name final_N_rl_part1
```

## 6. 폴더

| 경로 | 내용 |
|---|---|
| `env/` | SUMO 환경 (`sumo_env.py`), 회전교차로 지도 (`sumo_roundabout/`), 규칙 기반 강사 (`expert_driver.py`), 지도 만들기 패치들 |
| `algorithms/`, `utils/` | PPO, BC, 신경망, 버퍼, 평가 |
| `train.py`, `train_bc.py`, `pretrain_ppo_bc.py` | 강화학습 / 모방학습 |
| `collect_pomdp_data.py`, `data_check.py`, `export_csv.py` | 데이터 수집·점검 |
| `eval_parallel.py`, `eval_robust.py`, `test.py`, `record_video.py` | 평가·영상 |
| `run_new.sh` | ③ 강사 수집 → ④ BC → ⑤ DAgger 전체 과정 |
| `scripts/N_rl.sh` | ⑥ PPO (최종 모델은 `LR=2.5e-6`, 20만 스텝 체크포인트) |
| `scripts/collect_final_N.sh`, `scripts/robust_N.sh` | 제출 데이터 수집, 강건성 평가 |
| `models/final_rl/`, `models/dagger_best/` | 최종 PPO, 비교용 DAgger 최고 |
| `project_materials/` | 수업 자료 |

`scripts/` 의 셸 스크립트는 저희 기계의 폴더 경로를 씁니다. 다른 곳에서 돌릴 때는 맨 위 `cd` 줄을 고쳐 주세요.
