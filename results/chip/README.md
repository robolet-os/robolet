# Chip logs

Logs of the runs on the ESP32-S3 quoted in [docs/results.md](../../docs/results.md). Files from before 0.16 show the
old project name (esp32ros).

| File | Firmware | Run |
|---|---|---|
| `chip_20261127_070423.txt` | 0.16 | `bash test.sh lostquick`: 5 worlds, kidnapping + unmapped box; candidate check while turning spread over cycles (the measurements in the README) |
| `chip_20261127_001217.txt` | 0.15 | the same, candidate check while turning in one cycle |
| `chip_20261126_234433.txt` | 0.14 | the same, global search in one cycle |
| `ros_20261126_175757.txt` | 0.13 | `bash test.sh ros`: services, parameters, lifecycle, actions (33 checks) |
| `cli_20261126_201134.txt` | 0.14 | `bash test.sh cli`: the ros2-style command line (29 checks) |
