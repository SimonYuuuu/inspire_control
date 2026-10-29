"""Raw Modbus TCP client for the Inspire RH56DFTP.

Deliberately not built on pymodbus: the hand only needs FC03/FC16 and a strict
client library adds reconnect policy and response validation that get in the way
of a hard real-time loop.
"""

from __future__ import annotations

import socket
import struct
import threading
import time

from . import registers as reg

_MBAP = ">HHHB"
_MBAP_SIZE = 7
_WRITE_ACK_SIZE = 12
_RECONNECT_GRACE_S = 0.3


class HandError(RuntimeError):
    """Transport or protocol failure talking to the hand."""


class InspireHand:
    """Synchronous Modbus TCP connection to one dexterous hand.

    All public methods are serialised by an internal lock, so a control thread
    and a telemetry thread may share one instance without interleaving frames.
    """

    def __init__(self, host: str, port: int = 6000, unit: int = 1, timeout: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.unit = unit
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._tid = 0
        self._last_close = 0.0

    def connect(self) -> None:
        with self._lock:
            self._connect_locked()

    def _connect_locked(self) -> None:
        self._close_locked()
        # Reconnecting too fast yields a socket that connects and then drops.
        remaining = _RECONNECT_GRACE_S - (time.monotonic() - self._last_close)
        if 0 < remaining <= _RECONNECT_GRACE_S:
            time.sleep(remaining)
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except OSError as exc:
            raise HandError(f"cannot reach {self.host}:{self.port}: {exc}") from exc
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.timeout)
        self._sock = sock

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
                self._last_close = time.monotonic()

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def _next_tid(self) -> int:
        self._tid = (self._tid + 1) & 0xFFFF
        return self._tid

    def _recv_exact(self, size: int) -> bytes:
        assert self._sock is not None
        chunks: list[bytes] = []
        got = 0
        while got < size:
            part = self._sock.recv(size - got)
            if not part:
                raise HandError("hand closed the connection")
            chunks.append(part)
            got += len(part)
        return b"".join(chunks)

    def read_holding(self, address: int, count: int) -> list[int]:
        """FC03. Returns ``count`` raw 16-bit registers."""
        with self._lock:
            if self._sock is None:
                self._connect_locked()
            request = struct.pack(">HHHBBHH", self._next_tid(), 0, 6, self.unit, 3, address, count)
            try:
                self._sock.sendall(request)  # type: ignore[union-attr]
                _, protocol, length, _ = struct.unpack(_MBAP, self._recv_exact(_MBAP_SIZE))
                body = self._recv_exact(length - 1)
            except OSError as exc:
                self._close_locked()
                raise HandError(f"read {address}+{count} failed: {exc}") from exc
            if protocol != 0:
                raise HandError(f"unexpected protocol id {protocol}")
            if body[0] & 0x80:
                raise HandError(f"modbus exception {body[1]} reading {address}")
            if body[0] != 3 or body[1] != count * 2:
                raise HandError(f"malformed read response: {body.hex()}")
            return list(struct.unpack(f">{count}H", body[2:]))

    def write_multiple(self, address: int, values: list[int]) -> None:
        """FC16. The 12-byte acknowledgement is drained, never skipped.

        Leaving the ack in the socket backs up the hand's send buffer; its main
        loop then blocks in send() and stops servicing commands, while the
        client's own sendall() keeps succeeding. Looks like a hand freeze.
        """
        payload = struct.pack(f">{len(values)}H", *(v & 0xFFFF for v in values))
        with self._lock:
            if self._sock is None:
                self._connect_locked()
            header = struct.pack(
                ">HHHBBHHB", self._next_tid(), 0, 7 + len(payload), self.unit, 16, address, len(values), len(payload)
            )
            try:
                self._sock.sendall(header + payload)  # type: ignore[union-attr]
                ack = self._recv_exact(_WRITE_ACK_SIZE)
            except OSError as exc:
                self._close_locked()
                raise HandError(f"write {address} failed: {exc}") from exc
            if ack[7] & 0x80:
                raise HandError(f"modbus exception {ack[8]} writing {address}")
            if ack[7] != 16:
                raise HandError(f"malformed write ack: {ack.hex()}")

    def read_angles(self) -> list[int]:
        return self.read_holding(reg.ANGLE_ACT, reg.DOF_COUNT)

    def read_forces(self) -> list[int]:
        """Gripping force per DOF in grams. The hand encodes these as signed."""
        return [v - 0x10000 if v > 0x7FFF else v for v in self.read_holding(reg.FORCE_ACT, reg.DOF_COUNT)]

    def set_angles(self, targets: list[int | None]) -> None:
        """Write target angles. ``None`` leaves that DOF untouched."""
        if len(targets) != reg.DOF_COUNT:
            raise ValueError(f"expected {reg.DOF_COUNT} targets, got {len(targets)}")
        encoded = [reg.NO_ACTION if t is None else int(t) for t in targets]
        self.write_multiple(reg.ANGLE_SET, encoded)

    def __enter__(self) -> InspireHand:
        self.connect()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class SimulatedHand:
    """Drop-in stand-in for :class:`InspireHand` used by ``--dry-run``.

    Lets the whole mapping and safety chain be exercised without powering the
    hardware. Angles follow the setpoint instantly; forces are always zero.
    """

    def __init__(self, *_args: object, **_kwargs: object) -> None:
        self._angles = [reg.RAW_MAX] * reg.DOF_COUNT
        self.connected = True

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def read_angles(self) -> list[int]:
        return list(self._angles)

    def read_forces(self) -> list[int]:
        return [0] * reg.DOF_COUNT

    def set_angles(self, targets: list[int | None]) -> None:
        for i, value in enumerate(targets):
            if value is not None and value != reg.NO_ACTION:
                self._angles[i] = int(value)
