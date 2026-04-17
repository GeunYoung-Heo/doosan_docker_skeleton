"""
OnRobot Compute Box client for RG2-FT V2.

Command: Modbus TCP (port 502, Unit ID 65)
  FC16 write registers 2–4: [force_01N, width_01mm, control=1]
  force_01N  = int(force_n  * 10)   range 0–400  (0.0–40.0 N)
  width_01mm = int(width_mm * 10)   range 0–1100 (0.0–110.0 mm)

State: Socket.IO (EIO=4, HTTP long-polling transport)
  event: "message"  →  data["devices"][0]["variable"]["backpack"]
    width  (float): actual opening in mm
    grip   (bool):  object detected / grip force reached
    ready  (bool):  True when motion complete (busy = not ready)

No third-party dependencies — uses only Python stdlib.
"""

import json
import socket
import struct
import threading
import time
import urllib.request

_MODBUS_PORT = 502
_UNIT_ID = 65

_FORCE_MIN = 0.0
_FORCE_MAX = 40.0
_WIDTH_MIN = 0.0
_WIDTH_MAX = 110.0

# Modbus holding register addresses
_REG_FORCE   = 2   # target force   [0.1 N units]
_REG_WIDTH   = 3   # target width   [0.1 mm units]
_REG_CONTROL = 4   # write 1 to execute command


class ComputeBoxClient:
    """Modbus TCP (command) + Socket.IO (state) client for RG2-FT V2."""

    def __init__(self, host: str = "192.168.1.1", timeout: float = 3.0) -> None:
        self._host = host
        self._timeout = timeout
        self._base = f"http://{host}"
        self._state: dict = {"actual_width": 0.0, "busy": False, "gripped": False}
        self._state_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._sio_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def command(self, width_mm: float, force_n: float) -> None:
        """Send a grip/position command via Modbus TCP FC16.

        Writes [force, width, control=1] to registers 2–4 atomically.

        Args:
            width_mm: Target opening in mm [0.0 – 110.0].
            force_n:  Target grip force in N [0.0 – 40.0].

        Raises:
            ValueError: Parameters out of range.
            OSError: Compute Box unreachable or Modbus error.
        """
        if not (_WIDTH_MIN <= width_mm <= _WIDTH_MAX):
            raise ValueError(
                f"width_mm {width_mm} out of range [{_WIDTH_MIN}, {_WIDTH_MAX}]"
            )
        if not (_FORCE_MIN <= force_n <= _FORCE_MAX):
            raise ValueError(
                f"force_n {force_n} out of range [{_FORCE_MIN}, {_FORCE_MAX}]"
            )

        force_01 = int(round(force_n * 10))
        width_01 = int(round(width_mm * 10))
        self._modbus_fc16(start_reg=_REG_FORCE, values=[force_01, width_01, 1])

    def get_state(self) -> dict:
        """Return the last received gripper state (non-blocking, cached).

        Returns:
            dict:
                actual_width (float): Current opening in mm.
                busy         (bool):  True while motion is in progress.
                gripped      (bool):  True when object detected / grip reached.
        """
        with self._state_lock:
            return dict(self._state)

    def ping(self) -> bool:
        """Return True if the Compute Box Modbus TCP port is reachable."""
        try:
            with socket.create_connection(
                (self._host, _MODBUS_PORT), timeout=self._timeout
            ):
                pass
            return True
        except OSError:
            return False

    def close(self) -> None:
        """Stop the background Socket.IO thread."""
        self._stop.set()

    # ------------------------------------------------------------------
    # Modbus TCP (command path)
    # ------------------------------------------------------------------

    def _modbus_fc16(self, start_reg: int, values: list) -> None:
        """FC16 Write Multiple Registers — creates a fresh TCP connection per call."""
        count = len(values)
        byte_count = count * 2
        tid = 1
        pdu = struct.pack(
            f">BHHB{count}H", 0x10, start_reg, count, byte_count, *values
        )
        # MBAP header: tid(2) + protocol=0(2) + length(2) + unit_id(1)
        mbap = struct.pack(">HHHB", tid, 0, 1 + len(pdu), _UNIT_ID)

        with socket.create_connection(
            (self._host, _MODBUS_PORT), timeout=self._timeout
        ) as sock:
            sock.sendall(mbap + pdu)
            header = self._recv_exact(sock, 6)
            length = struct.unpack_from(">H", header, 4)[0]
            body = self._recv_exact(sock, length)

        # body[0]=unit_id, body[1]=FC, body[2]=exception_code (on error)
        if len(body) >= 2 and (body[1] & 0x80):
            exc_code = body[2] if len(body) > 2 else 0
            raise OSError(f"Modbus exception on FC16: exception code {exc_code:#04x}")

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise OSError("Connection closed by Compute Box")
            buf += chunk
        return buf

    # ------------------------------------------------------------------
    # Socket.IO background thread (state reading)
    # ------------------------------------------------------------------

    def _sio_loop(self) -> None:
        """Reconnecting loop — restarts the session on any error."""
        while not self._stop.is_set():
            try:
                self._sio_session()
            except Exception:
                if not self._stop.is_set():
                    time.sleep(2.0)

    def _sio_session(self) -> None:
        """One Socket.IO HTTP-polling session (EIO=4)."""
        handshake_url = f"{self._base}/socket.io/?EIO=4&transport=polling"
        req = urllib.request.Request(handshake_url, method="GET")
        with urllib.request.urlopen(req, timeout=self._timeout) as resp:
            raw = resp.read().decode()

        # EIO4 handshake packet: "0{...json...}"
        json_str = raw.lstrip("0123456789")
        if json_str.startswith("0"):
            json_str = json_str[1:]
        sid = json.loads(json_str)["sid"]
        poll_url = f"{self._base}/socket.io/?EIO=4&transport=polling&sid={sid}"

        req = urllib.request.Request(
            poll_url,
            data=b"40",
            method="POST",
            headers={"Content-Type": "text/plain"},
        )
        with urllib.request.urlopen(req, timeout=self._timeout) as resp:
            resp.read()

        while not self._stop.is_set():
            req = urllib.request.Request(poll_url, method="GET")
            with urllib.request.urlopen(req, timeout=30) as resp:
                raw = resp.read().decode()
            self._parse_packets(raw)

            req = urllib.request.Request(
                poll_url,
                data=b"3",
                method="POST",
                headers={"Content-Type": "text/plain"},
            )
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                resp.read()

    def _parse_packets(self, raw: str) -> None:
        segments = raw.split("\x1e") if "\x1e" in raw else [raw]
        for seg in segments:
            if not seg.startswith("42"):
                continue
            try:
                payload = json.loads(seg[2:])
            except json.JSONDecodeError:
                continue
            if not isinstance(payload, list) or len(payload) < 2:
                continue
            event_name, event_data = payload[0], payload[1]
            if event_name == "message" and isinstance(event_data, dict):
                self._update_state(event_data)

    def _update_state(self, data: dict) -> None:
        devices = data.get("devices", [])
        if not devices:
            return
        backpack = devices[0].get("variable", {}).get("backpack", {})
        if not backpack:
            return
        new_state = {
            "actual_width": float(backpack.get("width", 0.0)),
            "busy": not bool(backpack.get("ready", True)),
            "gripped": bool(backpack.get("grip", False)),
        }
        with self._state_lock:
            self._state = new_state
