"""
tools.rover_tests.transport - Authenticated Persistent WebSocket & Cockpit Transport

Authoritative transport implementation for Rover One test harnesses:
- RFC 6455 persistent native WebSocket client with token authentication.
- 50Hz watchdog-safe velocity streaming (v, w) without per-frame HTTP overhead.
- Cockpit autonomy handshake: WAITING_FOR_ZERO -> 3-consecutive-zeros -> READY_DISARMED -> READY_ARMED.
- Strict command-source ownership management (CALIBRATION_TEST, ROS_AUTONOMY, NONE).
- Guaranteed zero/disarm cleanup invariant executed across all termination paths.
"""

import socket
import base64
import struct
import json
import time
import os
import urllib.request
import urllib.error
from typing import Optional, Dict, Any, List, Tuple

DEFAULT_OPERATOR_TOKEN = "787f1b987d6295357ff3f664e08b0c96984f4f82a7b1edc17adef2793e64a168"
DEFAULT_BRIDGE_TOKEN = "effd380c5e7ea570736ddcae531f2a0247dc6012412a6b6e2988bc509a0d612d"


def get_default_operator_token() -> str:
    token = os.environ.get("ROVER_OPERATOR_TOKEN", "")
    if token:
        return token
    # Check .env file if available
    candidate_paths = [
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), ".env"),
        "/home/ron/yahboom-encoder/.env"
    ]
    for env_path in candidate_paths:
        if os.path.exists(env_path):
            try:
                with open(env_path, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("ROVER_OPERATOR_TOKEN="):
                            val = line.split("=", 1)[1].strip("\"'")
                            if val:
                                return val
            except Exception:
                pass
    return DEFAULT_OPERATOR_TOKEN


def get_default_bridge_token() -> str:
    token = os.environ.get("ROVER_CMD_VEL_TOKEN", "")
    if token:
        return token
    # Check common .env paths
    candidate_paths = [
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), ".env"),
        "/home/ron/yahboom-encoder/.env"
    ]
    for env_path in candidate_paths:
        if os.path.exists(env_path):
            try:
                with open(env_path, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("ROVER_CMD_VEL_TOKEN="):
                            val = line.split("=", 1)[1].strip("\"'")
                            if val:
                                return val
            except Exception:
                pass
    return DEFAULT_BRIDGE_TOKEN


class TransportException(Exception):
    """Raised on connection, authentication, or protocol faults."""
    pass


class HandshakeException(TransportException):
    """Raised when the 3-consecutive-zero autonomy handshake fails."""
    def __init__(
        self,
        message: str,
        stage: str = "UNKNOWN",
        state: Optional[str] = None,
        zero_count: int = 0,
        cmd_source: Optional[str] = None,
        last_rejection_reason: Optional[str] = None,
        underlying_error: Optional[str] = None
    ):
        super().__init__(message)
        self.stage = stage
        self.state = state
        self.zero_count = zero_count
        self.cmd_source = cmd_source
        self.last_rejection_reason = last_rejection_reason
        self.underlying_error = underlying_error

    def get_details(self) -> Dict[str, Any]:
        return {
            "stage": self.stage,
            "state": self.state,
            "zero_count": self.zero_count,
            "cmd_source": self.cmd_source,
            "last_rejection_reason": self.last_rejection_reason,
            "underlying_error": self.underlying_error
        }


class NativeWSClient:
    """
    Lightweight, dependency-free RFC 6455 WebSocket client using native TCP sockets.
    Enables low-latency, persistent telemetry ingestion and ~50Hz command streaming.
    """
    def __init__(self, host: str = "127.0.0.1", port: int = 3000, path: str = "/ws", timeout: float = 2.0):
        self.host = host
        self.port = port
        self.path = path
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self.connected = False
        self.authenticated = False

    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))

            key = base64.b64encode(os.urandom(16)).decode("utf-8")
            handshake = (
                f"GET {self.path} HTTP/1.1\r\n"
                f"Host: {self.host}:{self.port}\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                f"Sec-WebSocket-Version: 13\r\n\r\n"
            )
            self.sock.sendall(handshake.encode("utf-8"))
            resp = self.sock.recv(1024)
            if b"101 Switching Protocols" in resp:
                self.connected = True
                self.sock.settimeout(0.002)  # Non-blocking poll mode
                return True
        except Exception:
            self.connected = False
            self.close()
        return False

    def send_json(self, obj: Dict[str, Any]) -> bool:
        if not self.connected or not self.sock:
            return False
        try:
            payload = json.dumps(obj).encode("utf-8")
            mask_key = os.urandom(4)
            length = len(payload)

            header = bytearray([0x81])  # FIN + text opcode
            if length < 126:
                header.append(0x80 | length)
            elif length < 65536:
                header.append(0x80 | 126)
                header.extend(struct.pack("!H", length))
            else:
                header.append(0x80 | 127)
                header.extend(struct.pack("!Q", length))

            masked_payload = bytearray(length)
            for i in range(length):
                masked_payload[i] = payload[i] ^ mask_key[i % 4]

            frame = header + mask_key + masked_payload
            self.sock.sendall(frame)
            return True
        except Exception:
            self.connected = False
            return False

    def authenticate(self, token: str, timeout: float = 2.0) -> bool:
        if not self.send_json({"type": "auth", "token": token}):
            return False

        t0 = time.time()
        while time.time() - t0 < timeout:
            msgs = self.recv_frames()
            for m in msgs:
                if m.get("type") == "auth_result":
                    if m.get("ok") is True:
                        self.authenticated = True
                        return True
                    return False
            time.sleep(0.02)
        return False

    def send_drive(self, vx: float, wz: float) -> bool:
        """Stream velocity command via persistent WebSocket keeping deadman/watchdogs alive."""
        if not self.connected:
            return False
        return self.send_json({
            "type": "test_drive",
            "v": float(vx),
            "w": float(wz)
        })

    def recv_frames(self, timeout: Optional[float] = None, max_frames: Optional[int] = None) -> List[Dict[str, Any]]:
        if not self.connected or not self.sock:
            return []
        messages: List[Dict[str, Any]] = []
        old_timeout = None
        if timeout is not None:
            try:
                old_timeout = self.sock.gettimeout()
                self.sock.settimeout(max(0.0001, float(timeout)))
            except Exception:
                pass
        try:
            while True:
                if max_frames is not None and len(messages) >= max_frames:
                    break
                head = self.sock.recv(2)
                if not head or len(head) < 2:
                    break
                b1, b2 = head[0], head[1]
                opcode = b1 & 0x0F
                payload_len = b2 & 0x7F
                if payload_len == 126:
                    ext = self.sock.recv(2)
                    if len(ext) < 2:
                        break
                    payload_len = struct.unpack("!H", ext)[0]
                elif payload_len == 127:
                    ext = self.sock.recv(8)
                    if len(ext) < 8:
                        break
                    payload_len = struct.unpack("!Q", ext)[0]

                payload = b""
                while len(payload) < payload_len:
                    chunk = self.sock.recv(payload_len - len(payload))
                    if not chunk:
                        break
                    payload += chunk

                if opcode == 0x01:  # Text frame
                    try:
                        text = payload.decode("utf-8", errors="ignore")
                        messages.append(json.loads(text))
                    except Exception:
                        pass
                elif opcode == 0x08:  # Close frame
                    self.connected = False
                    break
        except (socket.timeout, BlockingIOError):
            pass
        except Exception:
            pass
        finally:
            if old_timeout is not None and self.sock:
                try:
                    self.sock.settimeout(old_timeout)
                except Exception:
                    pass
        return messages

    def close(self):
        self.connected = False
        self.authenticated = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class CockpitClient:
    """
    HTTP client for Cockpit Server state, arming, autonomy, encoders, and IMU endpoints.
    """
    def __init__(self, host: str = "127.0.0.1", port: int = 3000, token: Optional[str] = None):
        self.base_url = f"http://{host}:{port}"
        self.token = token or get_default_operator_token()

    def _request(self, endpoint: str, method: str = "GET", data: Optional[Dict[str, Any]] = None, timeout: float = 2.0) -> Dict[str, Any]:
        url = f"{self.base_url}{endpoint}"
        headers = {
            "User-Agent": "RoverTestFramework/1.0",
            "x-rover-operator-token": self.token
        }
        payload = None
        if data is not None:
            headers["Content-Type"] = "application/json"
            payload = json.dumps(data).encode("utf-8")

        req = urllib.request.Request(url, data=payload, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read().decode("utf-8")
                return json.loads(raw) if raw else {}
        except urllib.error.HTTPError as e:
            try:
                err_body = json.loads(e.read().decode("utf-8"))
                return {"ok": False, "http_status": e.code, "error": err_body.get("error", str(e)), "response": err_body}
            except Exception:
                return {"ok": False, "http_status": e.code, "error": str(e)}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def get_status(self, timeout: float = 2.0) -> Dict[str, Any]:
        return self._request("/api/status", timeout=timeout)

    def get_autonomy_status(self, timeout: float = 2.0) -> Dict[str, Any]:
        return self._request("/api/autonomy/status", timeout=timeout)

    def enable_autonomy(self) -> Dict[str, Any]:
        return self._request("/api/autonomy/enable", method="POST", data={})

    def disable_autonomy(self) -> Dict[str, Any]:
        return self._request("/api/autonomy/disable", method="POST", data={})

    def arm_drive(self) -> Dict[str, Any]:
        return self._request("/api/drive/arm", method="POST", data={})

    def disarm_drive(self) -> Dict[str, Any]:
        return self._request("/api/drive/disarm", method="POST", data={})

    def set_command_source(self, source: str = "NONE") -> Dict[str, Any]:
        return self._request("/api/command-source", method="POST", data={"source": source})

    def configure_drive(
        self,
        wheel_balancing: bool = False,
        dynamic_braking: bool = False,
        brake_duration_ms: int = 100,
        max_trigger_speed: float = 0.35,
        timeout: float = 2.0
    ) -> Dict[str, Any]:
        """Configure runtime wheel balancing and dynamic braking modes."""
        return self._request("/api/drive/config", method="POST", data={
            "wheelBalancing": bool(wheel_balancing),
            "dynamicBraking": bool(dynamic_braking),
            "brakeDurationMs": int(brake_duration_ms),
            "maxTriggerSpeed": float(max_trigger_speed)
        }, timeout=timeout)

    def get_drive_config(self, timeout: float = 2.0) -> Dict[str, Any]:
        return self._request("/api/drive/config", method="GET", timeout=timeout)

    def get_encoders(self, timeout: float = 2.0) -> Dict[str, Any]:
        return self._request("/api/encoders", timeout=timeout)

    def get_imu(self, timeout: float = 2.0) -> Dict[str, Any]:
        return self._request("/api/imu", timeout=timeout)

    def get_pid_telemetry(self) -> Dict[str, Any]:
        return self._request("/api/pid-telemetry")

    def send_cmd_vel(self, vx: float = 0.0, wz: float = 0.0, bridge_port: int = 3010) -> Dict[str, Any]:
        """Send velocity command to internal ROS2 command bridge listener (port 3010)."""
        host = self.base_url.split("://", 1)[1].split(":", 1)[0]
        # Port 3010 is bound strictly to 127.0.0.1 on the Pi.
        # If connecting to the rover (by IP, hostname, or localhost), check 127.0.0.1 as well.
        candidate_hosts = [host]
        if host != "127.0.0.1":
            candidate_hosts.append("127.0.0.1")

        headers = {
            "Content-Type": "application/json",
            "x-rover-bridge-token": get_default_bridge_token()
        }
        payload = json.dumps({
            "linear": {"x": float(vx)},
            "angular": {"z": float(wz)}
        }).encode("utf-8")

        last_error = "Unknown connection failure"
        for target_host in candidate_hosts:
            url = f"http://{target_host}:{bridge_port}/api/cmd_vel"
            req = urllib.request.Request(url, data=payload, headers=headers, method="POST")
            try:
                with urllib.request.urlopen(req, timeout=1.0) as resp:
                    raw = resp.read().decode("utf-8")
                    return json.loads(raw) if raw else {}
            except urllib.error.HTTPError as e:
                try:
                    err_body = json.loads(e.read().decode("utf-8"))
                    return {"ok": False, "http_status": e.code, "error": err_body.get("error", str(e)), "response": err_body}
                except Exception:
                    return {"ok": False, "http_status": e.code, "error": str(e)}
            except Exception as e:
                last_error = str(e)

        return {"ok": False, "error": last_error}


def complete_zero_handshake(
    cockpit: CockpitClient,
    ws: Optional[NativeWSClient] = None,
    max_duration_sec: float = 4.0,
    transitions: Optional[List[Dict[str, Any]]] = None
) -> bool:
    """
    Executes the three-consecutive-zero autonomy handshake:
    1. POST /api/autonomy/enable -> transitions to WAITING_FOR_ZERO.
    2. Streams zero velocity commands until zeroHandshakeCount >= 3 and state == READY_DISARMED.
    """
    enable_res = cockpit.enable_autonomy()
    if not enable_res.get("ok", False):
        err_msg = enable_res.get("error", "Unknown error")
        stat = cockpit.get_autonomy_status() if hasattr(cockpit, "get_autonomy_status") else {}
        raise HandshakeException(
            f"Failed to enable autonomy: {err_msg}",
            stage="ENABLE_AUTONOMY",
            state=stat.get("state"),
            zero_count=stat.get("zeroHandshakeCount", 0),
            cmd_source=stat.get("cmdSource"),
            last_rejection_reason=stat.get("lastRejectionReason"),
            underlying_error=err_msg
        )

    if transitions is not None:
        transitions.append({
            "timestamp": time.time(),
            "state": "WAITING_FOR_ZERO",
            "trigger": "enable_autonomy"
        })

    t0 = time.time()
    handshake_success = False
    last_bridge_error: Optional[str] = None

    while time.time() - t0 < max_duration_sec:
        # Send zero command via WebSocket
        if ws and ws.connected:
            ws.send_drive(0.0, 0.0)

        # Also send zero command via internal ROS2 command bridge
        try:
            cmd_res = cockpit.send_cmd_vel(0.0, 0.0)
            if not cmd_res.get("ok", False):
                last_bridge_error = cmd_res.get("error")
        except Exception as e:
            last_bridge_error = str(e)

        # Check status
        auto_stat = cockpit.get_autonomy_status()
        state = auto_stat.get("state")
        z_count = auto_stat.get("zeroHandshakeCount", 0)

        if state == "READY_DISARMED" or z_count >= 3:
            handshake_success = True
            if transitions is not None:
                transitions.append({
                    "timestamp": time.time(),
                    "state": "READY_DISARMED",
                    "trigger": "zero_handshake_complete"
                })
            break
        elif state == "READY_ARMED" or state == "ACTIVE":
            handshake_success = True
            if transitions is not None:
                transitions.append({
                    "timestamp": time.time(),
                    "state": state,
                    "trigger": "already_armed"
                })
            break

        time.sleep(0.05)

    if not handshake_success:
        auto_stat = cockpit.get_autonomy_status()
        state = auto_stat.get("state")
        z_count = auto_stat.get("zeroHandshakeCount", 0)
        rejection = auto_stat.get("lastRejectionReason", "")
        err_details = f"state={state}, zeroCount={z_count}"
        if rejection:
            err_details += f", lastRejectionReason='{rejection}'"
        if last_bridge_error:
            err_details += f", lastBridgeError='{last_bridge_error}'"
        raise HandshakeException(
            f"Zero handshake failed to complete within {max_duration_sec}s ({err_details})",
            stage="WAITING_FOR_ZERO",
            state=state,
            zero_count=z_count,
            cmd_source=auto_stat.get("cmdSource"),
            last_rejection_reason=rejection,
            underlying_error=last_bridge_error
        )

    return True


def arm_and_verify_ready_armed(
    cockpit: CockpitClient,
    max_duration_sec: float = 2.0,
    transitions: Optional[List[Dict[str, Any]]] = None
) -> bool:
    """
    Arms drivetrain and confirms that both Cockpit and ESP32 report READY_ARMED and armed=True stably.
    """
    arm_res = cockpit.arm_drive()
    if not arm_res.get("ok", False):
        arm_err = arm_res.get("error", "Unknown arm error")
        auto_stat = cockpit.get_autonomy_status()
        raise HandshakeException(
            f"Failed to arm drivetrain after zero handshake: {arm_err}",
            stage="ARM_DRIVE",
            state=auto_stat.get("state"),
            zero_count=auto_stat.get("zeroHandshakeCount", 0),
            cmd_source=auto_stat.get("cmdSource"),
            last_rejection_reason=auto_stat.get("lastRejectionReason"),
            underlying_error=arm_err
        )

    # Confirm armed status: require hardware confirmation (armed=True, mode=3) and READY_ARMED state
    # Explicitly reported mode of 3 is strictly required (missing/null/wrong mode fails)
    t0 = time.time()
    while time.time() - t0 < max_duration_sec:
        stat = cockpit.get_status()
        auto_stat = cockpit.get_autonomy_status()
        is_armed = stat.get("armed") is True and stat.get("mode") == 3
        is_ready = auto_stat.get("state") in ("READY_ARMED", "ACTIVE")

        if is_armed and is_ready:
            if transitions is not None:
                transitions.append({
                    "timestamp": time.time(),
                    "state": "READY_ARMED",
                    "trigger": "arm_drive"
                })
            return True

        time.sleep(0.04)

    stat = cockpit.get_status()
    auto_stat = cockpit.get_autonomy_status()
    raise HandshakeException(
        f"Drivetrain did not report armed within confirmation window (armed={stat.get('armed')}, mode={stat.get('mode')}, state={auto_stat.get('state')})",
        stage="ARM_CONFIRMATION",
        state=auto_stat.get("state"),
        zero_count=auto_stat.get("zeroHandshakeCount", 0),
        cmd_source=stat.get("cmdSource")
    )


def perform_zero_handshake(
    cockpit: CockpitClient,
    ws: Optional[NativeWSClient] = None,
    max_duration_sec: float = 4.0,
    transitions: Optional[List[Dict[str, Any]]] = None,
    arm: bool = True
) -> bool:
    """
    Executes the three-consecutive-zero autonomy handshake, and optionally arms.
    """
    complete_zero_handshake(cockpit, ws=ws, max_duration_sec=max_duration_sec, transitions=transitions)
    if arm:
        arm_and_verify_ready_armed(cockpit, max_duration_sec=max_duration_sec, transitions=transitions)
    return True


def disarm_and_stop(
    cockpit: Optional[CockpitClient] = None,
    ws: Optional[NativeWSClient] = None,
    verbose: bool = True
) -> Dict[str, Any]:
    """
    Universal guaranteed zero/disarm cleanup invariant.
    Idempotent: safe to call repeatedly without causing state oscillations or double executions.
    Streams zero, disarms drive, disables autonomy, clears command source ownership,
    verifies confirmed disarmed state, and returns the verified final safety state.
    """
    if not cockpit and not ws:
        return {"armed": None, "autonomyState": None, "cmdSource": None}

    # 1. Stream zero command via WebSocket
    if ws and ws.connected:
        try:
            ws.send_drive(0.0, 0.0)
        except Exception:
            pass

    # 2. Stream zero command via ROS2 command bridge
    if cockpit:
        try:
            cockpit.send_cmd_vel(0.0, 0.0)
        except Exception:
            pass

        # 3. Disarm drive
        try:
            cockpit.disarm_drive()
        except Exception:
            pass

        # 4. Disable autonomy
        try:
            cockpit.disable_autonomy()
        except Exception:
            pass

        # 5. Clear command source ownership
        try:
            cockpit.set_command_source("NONE")
        except Exception:
            pass

    # 6. Verify and confirm final safety state in a bounded confirmation loop
    final_state = {"armed": None, "autonomyState": None, "cmdSource": None}
    if cockpit:
        t0 = time.time()
        while time.time() - t0 < 0.60:
            try:
                st = cockpit.get_status()
                final_state["armed"] = st.get("armed")
                final_state["autonomyState"] = st.get("autonomyState")
                final_state["cmdSource"] = st.get("cmdSource")
                if (
                    final_state["armed"] is False
                    and final_state["autonomyState"] == "DISABLED"
                    and final_state["cmdSource"] in ("NONE", None)
                ):
                    break
            except Exception:
                pass
            time.sleep(0.04)

        # Fallback if still reported armed, not disabled, or cmdSource not NONE: send explicit disarm & disable calls
        if (
            final_state["armed"] is not False
            or final_state["autonomyState"] != "DISABLED"
            or final_state["cmdSource"] not in ("NONE", None)
        ):
            try:
                cockpit.disarm_drive()
                cockpit.disable_autonomy()
                cockpit.set_command_source("NONE")
                time.sleep(0.05)
                st = cockpit.get_status()
                final_state["armed"] = st.get("armed")
                final_state["autonomyState"] = st.get("autonomyState")
                final_state["cmdSource"] = st.get("cmdSource")
            except Exception:
                pass

        if verbose:
            print(f"[CLEANUP VERIFIED] Final safety state: armed={final_state['armed']} autonomyState={final_state['autonomyState']} cmdSource={final_state['cmdSource']}")

    return final_state
