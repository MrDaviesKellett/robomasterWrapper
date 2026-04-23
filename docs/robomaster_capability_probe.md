# RoboMaster Capability Probe

This probe script is designed for protocol discovery and feature mapping across RoboMaster control interfaces.

- Script: `scripts/robomaster_capability_probe.py`
- Output: JSON report (default `probe-results/robomaster_probe_report.json`)
- Safety: query-first by default; optional aggressive mode for state-changing probes

## What it probes

1. UDP broadcast listeners (discover robot announcements)
2. TCP port reachability scan on known RoboMaster ports
3. Plaintext SDK command channel probes (`command`, query-style commands, fuzzed query combinations)
4. Python SDK (`robomaster` / `robomaster-sdk-modern`) initialization and module/method introspection
5. Optional serial/UART probing via `pyserial`

## Recommended first run (safe)

```bash
python3 scripts/robomaster_capability_probe.py --host 192.168.2.1
```

For USB/RNDIS, try:

```bash
python3 scripts/robomaster_capability_probe.py --host 192.168.42.2
```

## Deeper run

Try more transport combinations and serial ports:

```bash
python3 scripts/robomaster_capability_probe.py \
  --host 192.168.2.1 \
  --try-all-conn-types \
  --try-all-proto-types \
  --serial \
  --serial-ports /dev/tty.usbserial-0001
```

## Aggressive mode

Aggressive mode includes state-changing probes (for example stream/audio toggles).

```bash
python3 scripts/robomaster_capability_probe.py --host 192.168.2.1 --aggressive
```

Use only when the robot is in a safe environment.

## Useful options

- `--skip-text-sdk`
- `--skip-tcp-scan`
- `--skip-broadcast`
- `--skip-python-sdk`
- `--scan-ports 40923,20020,30030`
- `--broadcast-ports 40926,40925`
- `--output probe-results/my_run.json`

## Reading the report

Each result contains:

- `category`: probe subsystem (`text_sdk`, `python_sdk`, `tcp_scan`, `udp_broadcast`, `serial`)
- `name`: probe step name
- `target`: endpoint or module
- `ok`: pass/fail
- `elapsed_ms`: latency
- `response` / `error`
- `metadata`: command payload, module method maps, getters, etc.

Use repeated runs (Wi-Fi vs USB, different firmware versions) and compare reports to identify hidden or transport-specific capabilities.
