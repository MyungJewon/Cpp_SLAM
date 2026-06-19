# 멀티세션 맵 병합 (Multi-Session Map Merge) 설계

작성일: 2026-06-19

## 목표

여러 bag 파일을 각각 SLAM한 뒤, 같은 구역을 지나는 세션들을 ScanContext 기반
장소 인식으로 정렬하여 **하나의 일관된 맵**으로 병합한다. 최종적으로 앱에서
여러 PLY를 자동 병합하는 기능의 토대이며, 향후 "이미지/스캔으로 사전맵에
리로컬라이징"의 기반이 된다.

## 배경 / 현 상태

- 단일 세션 SLAM은 완성도 있게 동작 (순수 LiDAR + 루프 클로저).
  - 검증: data_2(360°) 맵이 레퍼런스 B5 대비 X/Y/Z 모두 ~10% 이내, 겹침 영역
    RMSE ~27cm.
- 재사용 가능한 부품이 이미 존재:
  - `SubmapRegistration` — coarse-to-fine GICP, fitness/overlap/rmse 게이트.
  - `Submap` — anchor-local 점군 + anchor 포즈 + AABB.
  - `ScanContext` — 20링×60섹터 디스크립터, `scanContextDistance`가 (거리,
    bestShift) 반환. **검증 완료**: 실데이터에서 yaw 복원 오차 0°, 동일/상이
    장소 분리 4배 마진, top-1 검색 8/8.
  - `PoseGraph` — iSAM2 기반 포즈그래프.
  - `PlyParser` — PLY 입출력.

## 핵심 원리

서브맵 점은 **anchor-local** 좌표로 저장된다. 따라서 최적화된 anchor 포즈로
다시 변환하면 (세션 내 루프 보정이 반영된) 월드 위치가 복원된다. 세션 간
병합도 동일하게 anchor 포즈만 보정하면 점군은 자동으로 따라온다.

## 아키텍처 (2단계)

```
[1단계: 세션 저장]  slam <bag> <points> [imu] --save-session <dir>
    SLAM 완료(세션 내 루프 클로저 포함) 후 다음을 저장:
        <dir>/submap_0000.ply ... (anchor-local 점군)
        <dir>/poses.txt           (행마다: anchorId R00..R22 tx ty tz)
        <dir>/meta.txt            (세션명, 서브맵 수, voxel 크기 등)

[2단계: 병합]  mapmerge <dir1> <dir2> ... -o merged.ply [--save-session <out>]
    세션 로드 → ScanContext 후보검색 → GICP 정합 → 일관성 클러스터링
    → 통합 포즈그래프 최적화 → 병합 PLY (+ 선택: 새 세션 저장)
```

## 컴포넌트

| 컴포넌트 | 신규/재사용 | 책임 | 의존 |
|---|---|---|---|
| `Session` (struct) | 신규 | 한 세션의 서브맵 목록 + 메타 | Submap |
| `SessionIO` | 신규 | 세션 dir 저장/로드 | PlyParser, Submap |
| `ScanContext` | 재사용 | 후보 검색 + yaw 추정 (z 센터링 추가) | — |
| `SubmapRegistration` | 재사용 | GICP 정합 | VoxelGrid, ICP |
| `SessionMerger` | 신규 | 후보검색·클러스터링·그래프·병합 | 위 전부 |
| `PoseGraph` | 재사용 | 통합 그래프 최적화 | GTSAM |
| `mapmerge` (main) | 신규 | 2단계 CLI 진입점 | SessionMerger |
| `slam` main | 수정 | `--save-session` 플래그 | SessionIO |

## 데이터 흐름 (병합)

세션0을 기준 좌표계로 고정. 각 추가 세션 S에 대해:

1. **후보 검색** — S의 각 서브맵에 대해 기준(이미 병합된) 세션 서브맵들과
   `scanContextDistance`로 top-K 후보. `bestShift × (360/SECTORS)` = yaw 초기추정.
   - 디스크립터 계산 전 서브맵 z를 중앙값으로 센터링(세션 간 z datum 차 보정).
2. **GICP 정합** — 후보마다 `registerSubmaps(submap_S, submap_ref, guess)`.
   - guess: yaw 회전 + (anchor 위치 기반) 평행이동. anchor-local 정합.
   - fitness/overlap 게이트 통과만 채택.
3. **세션 변환 후보화** — 각 매칭 → `T_SR = T_anchorRef · rel · inv(T_anchorS)`
   (세션 S 월드 → 기준 월드 변환). 여기서 T_anchor*는 세션 저장 포즈.
4. **일관성 클러스터링** — 후보 T_SR 들을 회전+이동 거리로 클러스터링.
   최대 클러스터 = 견고한 정렬. 클러스터 크기 < `minClusterSize`(기본 3)이면
   **"세션 정렬 실패" 보고 후 해당 세션 제외**(쓰레기 병합 방지).
5. **통합 포즈그래프** — 노드 = 전 세션 anchor. 엣지:
   - 세션 내: 인접 anchor 간 BetweenFactor(저장 포즈에서 계산), 타이트 노이즈.
   - 세션 간: 클러스터 인라이어 loop 각각 BetweenFactor(confidence 가중).
   - 세션 S anchor 초기값 = `T_SR · 저장포즈`.
   - 최적화 → 보정된 anchor 포즈 (두 맵의 잔여 드리프트까지 흡수).
6. **출력** — 각 서브맵을 보정된 anchor 포즈로 변환·누적 → voxel 다운샘플
   → 통계적 아웃라이어 제거 → `merged.ply`. `--save-session` 시 새 세션으로도
   저장(N개 체이닝 가능).

## 좌표/규약

- 포즈 `Pose3D{R(행우선 3x3), t}`. 기존 PoseMath 규약 사용.
- BetweenFactor 측정 = `inv(T_from) · T_to` (기존 LoopCloser와 동일).
- `registerSubmaps`의 `relativePose`는 src(local)→dst(local) 변환 = `inv(T_dst)·T_src`
  형태로 일관(기존 검증된 규약 유지).

## 오류 처리

- 세션 로드 실패(파일 없음/파싱 오류) → 명확한 메시지 후 중단.
- 세션 간 일관 클러스터 미발견 → 해당 세션 제외하고 경고. 모든 추가 세션이
  실패하면 기준 세션만 출력.
- GICP 발산(비유한값) → 해당 후보 폐기(기존 SubmapRegistration이 처리).
- iSAM2 최적화 예외 → try/catch, 최적화 실패 시 클러스터 강체변환으로 폴백.

## 테스트 전략

- **단위**
  - `ScanContext`: 알려진 yaw로 회전한 점군의 bestShift 복원 (검증 완료).
  - `SessionIO`: 세션 저장 후 로드 왕복 — 점군/포즈 동일성.
  - 클러스터링: 다수 정상 + 소수 이상치 입력 시 다수 클러스터 선택.
- **통합 (핵심)**
  - bag 하나를 시간상 절반으로 쪼개 두 세션으로 저장 → `mapmerge` → 병합 결과를
    원본 단일세션 맵과 비교 (RMSE 하니스 재사용). 정렬·병합 정확성 회귀 가드.

## 비목표 (YAGNI)

- 3개 초과 동시 세션의 전역 최적 순서 탐색(순차 누적으로 충분).
- 이미지/리로컬라이징(별도 후속 프로젝트, 본 설계는 토대만).
- 압축 bag(lz4/bz2) 지원(기존 한계 유지).
- 실시간 멀티세션(오프라인 배치만).

## 단계별 산출물

1. `slam --save-session`로 세션 1개 저장/로드 왕복 확인.
2. `mapmerge` 골격: 세션 로드 + 단순 강체 병합(클러스터링까지).
3. 통합 포즈그래프 최적화 추가.
4. 아웃라이어 제거 + 새 세션 저장.
5. split-merge 통합 테스트로 회귀 검증.
