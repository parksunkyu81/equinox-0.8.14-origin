# TorqueCenterOffset Count 0 수정 요약

`Count`는 원시 샘플 수가 아니라 오프셋 저장 승인 횟수입니다.

이번 최종 소스에는 다음 두 가지 수정이 포함됩니다.

1. GM 직선 미세 조향에서 발생하는 약한 rate-limit을 허용
2. 최초 18~20초 빠른 학습 후, 새 40~60초 데이터로 정밀 학습

세부 기준과 진단 방법은 다음 문서를 확인합니다.

```text
TORQUE_CENTER_OFFSET_STAGED_LEARNING_KO.md
```

실시간 모니터:

```bash
cd /data/openpilot
python3 selfdrive/debug/center_offset_monitor.py
```
