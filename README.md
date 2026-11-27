# 실생활 시나리오 기반 복합 도심 환경 자율주행 (BC → DAgger → PPO)

자율주행 인공지능 및 제어 팀 프로젝트 (팀 율붕붕 / Yulbungbung)

- GitHub 저장소: https://github.com/venezuela-chocolate-milk/Yulbungbung
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
인천광역시 검단구 왕길동 일대는 하나의 경로에서 무신호 회전교차로 진입, 유턴, 불법 주정차 차량 우회 등 주변 교통에 따라 AV가 속도와 차로를 조절해야 하는 상황을 함께 평가할 수 있어 선정했습니다.

- 들를 곳 5곳: 매번 주차장에 들어가 빈 칸 옆에 2초 정차, 마지막 곳 정차 = 완주
- 일반 차량과 만나는 상황: 회전교차로 진입·양보, 신호 교차로 좌회전, 합류, 불법 주정차 구간 추월(차선변경), 긴급차 양보
- 어린이보호구역 (30 km/h, 단속 카메라)
- 안전 마스킹은 끔 (규칙으로 행동을 막아 주지 않음)

## 2. POMDP

| 항목 | 내용 |
|---|---|
| 관측 (163차원) | 기본 자기 정보 11, 신호 5, 차로·화살표 정보, **보이는 차만** (가림 반영) 최대 10대 × 12값 (긴급차 여부 포함), 정차 구역 3 + 가까운 빈 칸 1 + 단속 카메라 1. 강사 전용 정보·다른 차 의도·차선 단위 경로 정보는 넣지 않음 |
| 행동 | 가감속 (연속, Gaussian) + 차선변경 {왼쪽, 유지, 오른쪽} (범주형) |
| 보상 | 진행 보상은 제한속도 주행 시 스텝당 +0.1, 시간 비용은 스텝당 −0.02. 경유지 정차 +10, 최종 완주 +10. 충돌(−50), 막힘·경로 이탈(−50), 적신호 통과(−30) 시 에피소드 종료. 근접·멈춤·과속·차로 변경·저크·긴급차 진로 방해에는 추가 감점 (`env/roundabout_env_config.py`의 `URBAN["reward"]`) |
| 신경망 | 주변 차 10대 → DeepSets 인코더 (12→64, max+mean), 나머지 43차원과 합쳐 MLP |

## 3. 학습 과정

| 단계 | 방법 | 결과 (완주 % / 충돌 %, 100판) |
|---|---|---|
| ③ 데이터 | 규칙 기반 강사 1000판 (349만 행) + 잡음 넣은 강사(DART) 1000판 (233만 행) | 강사 완주 95.8% |
| ④ BC | 강사 데이터로 모방학습, seed 3개 | 최고 21 / 7 (500판: 22.2 / 9.8) |
| ⑤ DAgger | 모델이 운전하고 강사가 정답을 다는 라운드 반복, 누적 데이터로 다시 학습 | 최고 (3라운드 d3s0) 91 / 2 (500판: 89.6 / 3.6) |
| ⑥ PPO | DAgger 최고에서 출발, PPO + BC 보조 손실 (계수 0.5), 학습률 2.5e-6. 139만 스텝까지 학습하고, 체크포인트를 평가해 20만 704 스텝 모델을 고름 | **최종 모델** |

강화학습은 설정을 여러 개(A부터 F까지) 시험했고, 모두 20만에서 40만 스텝 사이에서 가장 좋고 그 뒤로 나빠졌습니다. 그래서 짧게 학습하고, 처음 쓰는 seed 로 다시 평가해서 골랐습니다 (운 좋은 체크포인트를 고르지 않으려고). 최종 실행(E)도 139만 스텝까지 돌렸지만 가장 좋은 것은 20만 704 스텝 체크포인트였습니다 (수업 원본 코드의 기본 학습량도 20만 스텝).

## 4. 결과 (기준 교통량, 마스킹 끔)

| 모델 | 500판 seed 7000 | 200판 seed 8000 | 100판 seed 5000 |
|---|---|---|---|
| **최종 PPO (E 20만)** | **92.4 / 4.6** | 89.5 / 6.0 | 92 / 3 |
| DAgger 최고 (d3s0) | 89.6 / 3.6 | 89.0 / 7.5 | 87 / 3 |

- 최종 PPO 는 완주가 2.8%p 높고 충돌이 1.0%p 높습니다. 500판 오차(완주 ±2.7, 충돌 ±1.7) 안이라 **DAgger 최고를 확실히 넘었다고 말할 수는 없고, 비슷한 수준**입니다.
- 더 큰 비교(아래 강건성, 같은 5,000판)에서는 최종 PPO 의 완주율이 DAgger 최고보다 1.7%p 높았고, 이 차이는 우연으로 보기 어렵습니다.
- 충돌 5% 기준: 500판 4.6%. 200판 seed 8000 에서는 6.0% 로 경계에 가깝습니다 (같은 200판에서 DAgger 최고는 7.5%).

### 강건성 (처음 보는 seed 5개 × 1000판, 판마다 긴급차를 만날 때까지 투입)

| Seed | 판 수 | 완주 | 충돌 | 신호위반 | 막힘·이탈 | 시간초과 |
|---|---|---|---|---|---|---|
| 100,000 | 1,000 | 90.8% | 4.9% | 2.3% | 1.8% | 0.2% |
| 200,000 | 1,000 | 89.6% | 5.2% (≥5%) | 2.7% | 2.1% | 0.4% |
| 300,000 | 1,000 | 90.8% | 5.0% (≥5%) | 2.5% | 1.1% | 0.6% |
| 400,000 | 1,000 | 91.3% | 4.3% | 1.7% | 2.3% | 0.4% |
| 500,000 | 1,000 | 90.9% | 4.3% | 2.0% | 2.2% | 0.6% |
| **합계** | **5,000** | **90.7%** | **4.74%** | 2.2% | 1.9% | 0.4% |

합계 충돌률 4.74% (95% 범위 약 4.2~5.3%). 평균은 5% 미만이지만 기준선에 가깝고, seed 2개는 5.0% 이상이었습니다. seed별 차이는 1,000판 표본 오차(±1.4%p) 범위입니다. 강건성 평가는 일반 평가와 같은 환경(판마다 AV 가 긴급차를 한 번 만날 때까지 뒤에서 투입, 119 정기 출동과 만나도 인정)에서 학습·모델 선정에 쓰지 않은 seed 로 판 수를 늘린 것입니다. 결과 파일 기준 약 97% 의 판에 긴급차가 1대 이상 투입됐습니다. 명령의 `--ev-prob 0.5` 는 이 '만날 때까지 투입' 설정이 켜진 환경에서는 효과가 없습니다.

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

## 5. 실행 방법 (Environment / Training / Evaluation / Data Collection)

Ubuntu 22.04, Python 3.10 기준. 모든 명령은 저장소 맨 위 폴더에서 실행합니다.

```bash
# 'No module named venv' / 'ensurepip is not available' 가 나오면 먼저: sudo apt install python3-venv
python3 -m venv ~/sumo-venv
~/sumo-venv/bin/pip install -r requirements-sumo-driving.txt
```

### Environment

| 파일 | 내용 |
|---|---|
| `env/sumo_env.py` | Gymnasium 환경. 관측 163차원 (센서 범위 안에 보이는 차만), 행동 [가감속, 차선변경], 보상, 종료 조건 |
| `env/maps.py` | `make_env("roundabout")` 로 환경 생성 |
| `env/sumo_roundabout/` | 인천 검단구 왕길동 일대 SUMO 지도 (도로, 신호, 교통량, 주차장, 건물) |
| `env/mom_route.py`, `env/mom_driving.py` | 엄마의 하루 경로 (5개 방문 지점), 주차장 정차·긴급차 양보·어린이보호구역 판정 |
| `env/mdp_config.py`, `env/obs_layout.py` | 관측·행동·보상 설정, 관측 벡터 안의 위치 |
| `env/expert_driver.py` | 규칙 기반 강사 (BC·DAgger 정답 라벨) |
| `env/roundabout_*.py` | 지도를 만든 패치들 |

```bash
# 지도만 sumo-gui 로 보기 (학습 없이, ▶ 를 누르면 차가 움직임)
~/sumo-venv/bin/python view_roundabout.py
```

### Training

```bash
# ③ 강사 1000판 + DART 1000판 수집 → ④ BC (seed 3개) → ⑤ DAgger 라운드 (결과: data/mom_N_*.npz, results/bc_N_*)
WAIT_MAX=0 bash run_new.sh 20         # 20 = 병렬 작업자 수. WAIT_MAX=0: 다른 PC 의 DART 데이터를 기다리지 않고 바로 직접 수집 (기본 60분 대기)

# ⑥ PPO: DAgger 최고 (d3s0, models/dagger_best/model.pt) 에서 시작, BC 보조 손실 0.5, 학습률 2.5e-6 (최종 실행 E)
LR=2.5e-6 bash scripts/N_rl.sh models/dagger_best/model.pt rl_NE 1600000
#   → results/rl_NE/model_*.pt 체크포인트. 평가로 고른 200,704 스텝 체크포인트가 최종 모델 (models/final_rl/model.pt)
```

⑥ `scripts/N_rl.sh` 는 ③~⑤ 에서 만든 `data/mom_N_*.npz` 를 BC 보조 손실 데이터로 쓰므로 `run_new.sh` 를 먼저 끝내야 합니다. 시작 모델은 최종 실행에 쓴 DAgger d3s0 (`models/dagger_best/model.pt`, GitHub 저장소에 있음) 입니다. 처음부터 다시 돌렸다면 `results/best_N.txt` 에 적힌 DAgger 최고 모델로 시작하면 됩니다 (저희는 후보들을 200판으로 다시 평가해 d3s0 를 골랐습니다). 최종 실행 E 는 다른 PC 에서 모은 DAgger 1라운드 데이터 (`mom_N_dagM_r5.npz`) 도 함께 썼으므로, 처음부터 다시 돌리면 같은 방법이지만 숫자가 똑같이 나오지는 않습니다.

BC 를 직접 돌릴 때는 `pretrain_ppo_bc.py` 를 쓰고 옵션은 `run_new.sh` 안의 명령과 같습니다 (예: `~/sumo-venv/bin/python pretrain_ppo_bc.py --data data/mom_N_expert.npz data/mom_N_dart.npz --hold-target -0.1 --hidden 512 512 512 --seed 0 --out results/bc_N_seed0`). PPO 는 `train.py` 이고 옵션은 `scripts/N_rl.sh` 안의 명령과 같습니다. `train_bc.py` 는 수업 원본 BC 스크립트로 옵션이 달라 (`--hold-target`, `--hidden` 없음) 이 프로젝트 결과에는 쓰지 않았습니다.

### Evaluation

```bash
# 최종 모델 주행 보기 (sumo-gui 창이 뜨면 ▶ 를 누름)
#   고정 시드: 옵션 없이 실행하면 기본 시드 1. 최종 모델이 이 판을 완주하는 것을 확인함 (언제 돌려도 같은 판)
~/sumo-venv/bin/python test.py models/final_rl/model.pt --route mom --episodes 1
#   랜덤 시드: 실행할 때마다 다른 판 (평가마다 완주율 89.5~92.4%, 처음 보는 seed 5,000판 기준 90.7%. 일부 판은 충돌·막힘으로 끝날 수 있음)
#   시드 지정: 출력된 시작 시드를 --seed 로 주면 같은 판 재현
~/sumo-venv/bin/python test.py models/final_rl/model.pt --route mom --episodes 1 --random-seed
~/sumo-venv/bin/python test.py models/final_rl/model.pt --route mom --episodes 1 --seed 1234

# 최종 모델 평가 (작업자 수는 CPU 에 맞게)
~/sumo-venv/bin/python eval_parallel.py models/final_rl/model.pt --episodes 100 --workers 8 --seed 5000

# 강건성 평가 (판마다 seed 고정, 판마다 긴급차를 만날 때까지 투입. 결과와 같은 명령이며 --ev-prob 0.5 는 이 설정에서 효과 없음)
~/sumo-venv/bin/python eval_robust.py models/final_rl/model.pt --episodes 1000 --workers 8 --seed 100000 --ev-prob 0.5

# 주행 영상 (sumo-gui 필요)
~/sumo-venv/bin/python record_video.py --model models/final_rl/model.pt --route mom --fps 20
```

### Data Collection

```bash
# 최종 모델로 주행 데이터 수집 (제출 데이터와 같은 설정; 묶음 p 는 --seed 1000000*p)
~/sumo-venv/bin/python collect_pomdp_data.py --policy ppo --model models/final_rl/model.pt \
  --episodes 700 --workers 8 --seed 1000000 --ev-mode eval --state-radius 60 --name final_N_rl_part1

# 품질 점검 (관측 163차원, NaN/Inf, 범위, 중복 행)
~/sumo-venv/bin/python data_check.py data/final_N_rl_part1.npz

# JSONL → 수업 예시와 같은 칼럼의 CSV
~/sumo-venv/bin/python export_csv.py data/final_N_rl_part1.jsonl.gz --rows 15
```

`scripts/robust_N.sh` 는 강건성 평가 5개 seed 를 한 번에 돌리는 스크립트입니다. `scripts/collect_final_N.sh` 는 최종 데이터 4묶음 수집에 쓴 팀 장비용 일괄 스크립트입니다 (수집 뒤 팀 장비로 SSH 전송까지 함).

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
| `tests/` | 지도 형상 검사 (`~/sumo-venv/bin/python -m unittest tests.test_roundabout_geometry`). 지도를 그릴 때 만든 검사라, 이후 지도를 고치면서 기준과 달라진 5개 항목(교차로·주차장 안 짧은 연결 차로의 3.2 m 폭, 남서 T자 확장부 연결점 0.25 mm 어긋남, 점선 2개 길이, 정지선 간격, 이름이 바뀐 T자 교차로)은 '알려진 차이'(expectedFailure)로 표시했습니다. 시뮬레이션 주행에는 영향이 없고 지도는 결과와 같게 그대로 둡니다 |
| `models/final_rl/`, `models/dagger_best/` | 최종 PPO, 비교용 DAgger 최고 |
| `project_materials/` | 수업 자료 |

`run_new.sh`, `scripts/N_rl.sh`, `scripts/robust_N.sh` 는 저장소 폴더를 기준으로 움직이므로 받은 위치와 상관없이 실행됩니다 (파이썬은 `~/sumo-venv`). `scripts/collect_final_N.sh` 는 팀 장비 간 전송(SSH)까지 포함한 일괄 실행용이라, 개인 PC 에서 데이터를 모을 때는 5장 Data Collection 의 `collect_pomdp_data.py` 명령을 쓰세요.
