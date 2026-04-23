#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


DEFAULT_HOSTS = ["192.168.2.1", "192.168.42.2"]
DEFAULT_CONTROL_PORT = 40923
DEFAULT_SCAN_PORTS = [
    40923,
    40924,
    40925,
    40926,
    20020,
    30030,
]
DEFAULT_BROADCAST_PORTS = [40926, 40925]


@dataclass
class ProbeResult:
    category: str
    name: str
    target: str
    ok: bool
    elapsed_ms: int
    response: str | None = None
    error: str | None = None
    metadata: dict[str, Any] | None = None


class Reporter:
    def __init__(self) -> None:
        self.results: list[ProbeResult] = []

    def add(self, result: ProbeResult) -> None:
        self.results.append(result)
        status = "OK" if result.ok else "FAIL"
        line = f"[{status}] {result.category}:{result.name} ({result.target}) {result.elapsed_ms}ms"
        if result.response:
            line += f" -> {result.response.strip()}"
        if result.error:
            line += f" | {result.error}"
        print(line)

    def summary(self) -> dict[str, Any]:
        success = sum(1 for item in self.results if item.ok)
        failed = len(self.results) - success
        by_category: dict[str, dict[str, int]] = {}
        for item in self.results:
            bucket = by_category.setdefault(item.category, {"ok": 0, "fail": 0})
            if item.ok:
                bucket["ok"] += 1
            else:
                bucket["fail"] += 1
        return {
            "total": len(self.results),
            "ok": success,
            "fail": failed,
            "by_category": by_category,
        }

    def to_payload(self, args: argparse.Namespace) -> dict[str, Any]:
        return {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "script": "robomaster_capability_probe.py",
            "args": vars(args),
            "summary": self.summary(),
            "results": [asdict(item) for item in self.results],
        }


class TextSdkProbe:
    def __init__(self, host: str, port: int, timeout: float, reporter: Reporter) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.reporter = reporter
        self.sock: socket.socket | None = None

    def connect(self) -> bool:
        started = time.monotonic()
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            self.sock.settimeout(self.timeout)
            elapsed = int((time.monotonic() - started) * 1000)
            self.reporter.add(
                ProbeResult(
                    category="text_sdk",
                    name="connect",
                    target=f"{self.host}:{self.port}",
                    ok=True,
                    elapsed_ms=elapsed,
                    response="connected",
                )
            )
            return True
        except Exception as exc:
            elapsed = int((time.monotonic() - started) * 1000)
            self.reporter.add(
                ProbeResult(
                    category="text_sdk",
                    name="connect",
                    target=f"{self.host}:{self.port}",
                    ok=False,
                    elapsed_ms=elapsed,
                    error=str(exc),
                )
            )
            return False

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None

    def send(self, command: str, name: str = "command") -> ProbeResult:
        started = time.monotonic()
        if self.sock is None:
            result = ProbeResult(
                category="text_sdk",
                name=name,
                target=f"{self.host}:{self.port}",
                ok=False,
                elapsed_ms=0,
                error="not connected",
                metadata={"command": command},
            )
            self.reporter.add(result)
            return result

        payload = command.strip()
        if not payload.endswith(";"):
            payload += ";"

        try:
            self.sock.sendall(payload.encode("utf-8"))
            data = self.sock.recv(4096)
            response = data.decode("utf-8", errors="replace").strip()
            ok = bool(response) and not response.lower().startswith("error")
            elapsed = int((time.monotonic() - started) * 1000)
            result = ProbeResult(
                category="text_sdk",
                name=name,
                target=f"{self.host}:{self.port}",
                ok=ok,
                elapsed_ms=elapsed,
                response=response,
                metadata={"command": command},
            )
            self.reporter.add(result)
            return result
        except Exception as exc:
            elapsed = int((time.monotonic() - started) * 1000)
            result = ProbeResult(
                category="text_sdk",
                name=name,
                target=f"{self.host}:{self.port}",
                ok=False,
                elapsed_ms=elapsed,
                error=str(exc),
                metadata={"command": command},
            )
            self.reporter.add(result)
            return result


def build_safe_query_commands() -> list[tuple[str, str]]:
    commands: list[tuple[str, str]] = [
        ("command", "enter_sdk_mode"),
        ("version ?", "version"),
        ("robot mode ?", "robot_mode"),
        ("robot sn ?", "robot_sn"),
        ("robot battery ?", "robot_battery"),
        ("chassis speed ?", "chassis_speed"),
        ("chassis position ?", "chassis_position"),
        ("gimbal attitude ?", "gimbal_attitude"),
        ("armor sensitivity ?", "armor_sensitivity"),
    ]

    objects = [
        "robot",
        "chassis",
        "gimbal",
        "blaster",
        "led",
        "vision",
        "stream",
        "audio",
        "armor",
        "sensor",
        "sensor_adapter",
        "servo",
        "gripper",
        "robotic_arm",
        "uart",
    ]
    verbs = [
        "mode",
        "speed",
        "position",
        "status",
        "version",
        "battery",
        "attitude",
        "sensitivity",
        "distance",
        "color",
        "stream",
    ]
    for obj in objects:
        commands.append((f"{obj} ?", f"fuzz_{obj}_q"))
        for verb in verbs:
            commands.append((f"{obj} {verb} ?", f"fuzz_{obj}_{verb}_q"))

    commands.append(("quit", "exit_sdk_mode"))
    return commands


def build_aggressive_commands() -> list[tuple[str, str]]:
    return [
        ("command", "enter_sdk_mode_aggressive"),
        ("stream on", "stream_on"),
        ("stream off", "stream_off"),
        ("audio on", "audio_on"),
        ("audio off", "audio_off"),
        ("quit", "exit_sdk_mode_aggressive"),
    ]


def probe_text_sdk(host: str, args: argparse.Namespace, reporter: Reporter) -> None:
    probe = TextSdkProbe(host=host, port=args.control_port, timeout=args.timeout, reporter=reporter)
    if not probe.connect():
        return
    try:
        for command, name in build_safe_query_commands():
            probe.send(command, name=name)
            time.sleep(args.inter_command_delay)
        if args.aggressive:
            for command, name in build_aggressive_commands():
                probe.send(command, name=name)
                time.sleep(args.inter_command_delay)
    finally:
        probe.close()


def probe_tcp_ports(host: str, ports: Iterable[int], timeout: float, reporter: Reporter) -> None:
    for port in ports:
        started = time.monotonic()
        try:
            sock = socket.create_connection((host, port), timeout=timeout)
            sock.settimeout(timeout)
            banner = b""
            try:
                banner = sock.recv(512)
            except Exception:
                banner = b""
            sock.close()
            elapsed = int((time.monotonic() - started) * 1000)
            reporter.add(
                ProbeResult(
                    category="tcp_scan",
                    name="port_probe",
                    target=f"{host}:{port}",
                    ok=True,
                    elapsed_ms=elapsed,
                    response=banner.decode("utf-8", errors="replace") if banner else "open",
                )
            )
        except Exception as exc:
            elapsed = int((time.monotonic() - started) * 1000)
            reporter.add(
                ProbeResult(
                    category="tcp_scan",
                    name="port_probe",
                    target=f"{host}:{port}",
                    ok=False,
                    elapsed_ms=elapsed,
                    error=str(exc),
                )
            )


def probe_udp_broadcast(ports: Iterable[int], timeout: float, reporter: Reporter) -> None:
    for port in ports:
        started = time.monotonic()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        try:
            sock.bind(("", port))
            data, addr = sock.recvfrom(2048)
            elapsed = int((time.monotonic() - started) * 1000)
            reporter.add(
                ProbeResult(
                    category="udp_broadcast",
                    name="listen",
                    target=f"0.0.0.0:{port}",
                    ok=True,
                    elapsed_ms=elapsed,
                    response=data.decode("utf-8", errors="replace").strip(),
                    metadata={"source": f"{addr[0]}:{addr[1]}"},
                )
            )
        except Exception as exc:
            elapsed = int((time.monotonic() - started) * 1000)
            reporter.add(
                ProbeResult(
                    category="udp_broadcast",
                    name="listen",
                    target=f"0.0.0.0:{port}",
                    ok=False,
                    elapsed_ms=elapsed,
                    error=str(exc),
                )
            )
        finally:
            sock.close()


def probe_python_sdk(args: argparse.Namespace, reporter: Reporter) -> None:
    started = time.monotonic()
    try:
        from robomaster import robot as sdk_robot  # type: ignore
    except Exception as exc:
        reporter.add(
            ProbeResult(
                category="python_sdk",
                name="import",
                target="robomaster.robot",
                ok=False,
                elapsed_ms=int((time.monotonic() - started) * 1000),
                error=str(exc),
            )
        )
        return

    reporter.add(
        ProbeResult(
            category="python_sdk",
            name="import",
            target="robomaster.robot",
            ok=True,
            elapsed_ms=int((time.monotonic() - started) * 1000),
            response="imported",
        )
    )

    conn_types = ["ap", "sta", "rndis"] if args.try_all_conn_types else [args.sdk_conn_type]
    proto_types = ["udp", "tcp"] if args.try_all_proto_types else [args.sdk_proto_type]

    for conn_type in conn_types:
        for proto_type in proto_types:
            name = f"init_{conn_type}_{proto_type}"
            st = time.monotonic()
            ep = None
            try:
                ep = sdk_robot.Robot()
                ok = ep.initialize(conn_type=conn_type, proto_type=proto_type)
                elapsed = int((time.monotonic() - st) * 1000)
                if ok is False:
                    reporter.add(
                        ProbeResult(
                            category="python_sdk",
                            name=name,
                            target="robot.initialize",
                            ok=False,
                            elapsed_ms=elapsed,
                            response="initialize returned False",
                        )
                    )
                    continue

                modules = {}
                for attr in [
                    "chassis",
                    "gimbal",
                    "vision",
                    "camera",
                    "led",
                    "blaster",
                    "robotic_arm",
                    "gripper",
                    "servo",
                    "sensor",
                    "sensor_adaptor",
                    "armor",
                    "uart",
                ]:
                    obj = getattr(ep, attr, None)
                    if obj is None:
                        continue
                    methods = [
                        item
                        for item in dir(obj)
                        if not item.startswith("_") and callable(getattr(obj, item, None))
                    ]
                    modules[attr] = methods

                meta: dict[str, Any] = {"modules": modules}
                for getter in ["get_version", "get_sn", "get_robot_mode"]:
                    fn = getattr(ep, getter, None)
                    if callable(fn):
                        try:
                            meta[getter] = fn()
                        except Exception as exc:  # pragma: no cover - hardware dependent
                            meta[getter] = f"ERROR: {exc}"

                reporter.add(
                    ProbeResult(
                        category="python_sdk",
                        name=name,
                        target="robot.initialize",
                        ok=True,
                        elapsed_ms=elapsed,
                        response="initialized",
                        metadata=meta,
                    )
                )
            except Exception as exc:
                elapsed = int((time.monotonic() - st) * 1000)
                reporter.add(
                    ProbeResult(
                        category="python_sdk",
                        name=name,
                        target="robot.initialize",
                        ok=False,
                        elapsed_ms=elapsed,
                        error=str(exc),
                    )
                )
            finally:
                if ep is not None:
                    try:
                        ep.close()
                    except Exception:
                        pass


def probe_serial(args: argparse.Namespace, reporter: Reporter) -> None:
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except Exception as exc:
        reporter.add(
            ProbeResult(
                category="serial",
                name="import_pyserial",
                target="serial",
                ok=False,
                elapsed_ms=0,
                error=str(exc),
            )
        )
        return

    ports: list[str] = []
    if args.serial_ports:
        ports = [item.strip() for item in args.serial_ports.split(",") if item.strip()]
    else:
        ports = [item.device for item in list_ports.comports()]

    if not ports:
        reporter.add(
            ProbeResult(
                category="serial",
                name="enumerate",
                target="ports",
                ok=False,
                elapsed_ms=0,
                error="no serial ports found",
            )
        )
        return

    for port in ports:
        started = time.monotonic()
        handle = None
        try:
            handle = serial.Serial(port=port, baudrate=args.serial_baud, timeout=args.timeout)
            handle.write(b"command;\n")
            time.sleep(0.15)
            reply = handle.read(256).decode("utf-8", errors="replace").strip()
            elapsed = int((time.monotonic() - started) * 1000)
            reporter.add(
                ProbeResult(
                    category="serial",
                    name="probe",
                    target=port,
                    ok=bool(reply),
                    elapsed_ms=elapsed,
                    response=reply or "no response",
                    metadata={"baud": args.serial_baud},
                )
            )
        except Exception as exc:
            elapsed = int((time.monotonic() - started) * 1000)
            reporter.add(
                ProbeResult(
                    category="serial",
                    name="probe",
                    target=port,
                    ok=False,
                    elapsed_ms=elapsed,
                    error=str(exc),
                    metadata={"baud": args.serial_baud},
                )
            )
        finally:
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass


def parse_ports(raw: str | None, fallback: list[int]) -> list[int]:
    if not raw:
        return fallback
    result: list[int] = []
    for chunk in raw.split(","):
        value = chunk.strip()
        if not value:
            continue
        result.append(int(value))
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe RoboMaster control interfaces and capability exposure (safe query mode by default)."
    )
    parser.add_argument("--host", default=None, help="Robot host. Defaults to probing known AP/RNDIS hosts.")
    parser.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    parser.add_argument("--timeout", type=float, default=1.5)
    parser.add_argument("--inter-command-delay", type=float, default=0.05)
    parser.add_argument("--scan-ports", default=",".join(str(p) for p in DEFAULT_SCAN_PORTS))
    parser.add_argument("--broadcast-ports", default=",".join(str(p) for p in DEFAULT_BROADCAST_PORTS))
    parser.add_argument("--skip-text-sdk", action="store_true")
    parser.add_argument("--skip-tcp-scan", action="store_true")
    parser.add_argument("--skip-broadcast", action="store_true")
    parser.add_argument("--skip-python-sdk", action="store_true")
    parser.add_argument("--serial", action="store_true", help="Enable pyserial/UART probing.")
    parser.add_argument("--serial-ports", default=None, help="Comma-separated serial ports.")
    parser.add_argument("--serial-baud", type=int, default=115200)
    parser.add_argument("--aggressive", action="store_true", help="Enable state-changing probes (stream/audio toggles).")
    parser.add_argument("--sdk-conn-type", default="ap", choices=["ap", "sta", "rndis"])
    parser.add_argument("--sdk-proto-type", default="udp", choices=["udp", "tcp"])
    parser.add_argument("--try-all-conn-types", action="store_true")
    parser.add_argument("--try-all-proto-types", action="store_true")
    parser.add_argument("--output", default="probe-results/robomaster_probe_report.json")
    return parser


def main(argv: list[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    reporter = Reporter()

    hosts = [args.host] if args.host else DEFAULT_HOSTS
    scan_ports = parse_ports(args.scan_ports, DEFAULT_SCAN_PORTS)
    broadcast_ports = parse_ports(args.broadcast_ports, DEFAULT_BROADCAST_PORTS)

    print("RoboMaster capability probe started")
    print(f"Hosts: {hosts}")

    if not args.skip_broadcast:
        probe_udp_broadcast(broadcast_ports, timeout=args.timeout, reporter=reporter)

    for host in hosts:
        if not args.skip_tcp_scan:
            probe_tcp_ports(host, ports=scan_ports, timeout=args.timeout, reporter=reporter)
        if not args.skip_text_sdk:
            probe_text_sdk(host, args=args, reporter=reporter)

    if not args.skip_python_sdk:
        probe_python_sdk(args, reporter)

    if args.serial:
        probe_serial(args, reporter)

    payload = reporter.to_payload(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print("\nProbe complete")
    print(json.dumps(payload["summary"], indent=2))
    print(f"Report written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
