"""CLI command serialization and streaming-response collection.

The firmware CLI has no request-id or echo: a command is sent as ASCII text
terminated by \\r\\n, and its response is one or more lines prefixed with
``[CLI] `` and terminated by \\r\\n. There is no explicit end-of-response
marker. To match a command to its response we:

1. Serialize all outbound commands through a single in-process queue so the
   command/response stream on the wire is always strictly ordered.
2. After sending a command, collect every ``[CLI]`` line until the wire goes
   quiet for ``response_idle_ms`` (default 150 ms). That window is treated as
   the end of the response.

This module does not touch the serial port directly. The serial link layer
calls ``on_ascii_line()`` with each decoded line; this module owns the small
state machine that pairs lines to commands.

Failure detection: a response line containing ``[CLI] err:`` marks the
command as failed (is_error=True).
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Awaitable, Callable, Optional


SendBytesFn = Callable[[bytes], Awaitable[None]]
# async send_fn(bytes) -> None  -- writes to the serial port


@dataclass
class CommandResult:
    text: str               # the raw response text (may be multi-line, joined by \n)
    lines: list             # list[str], one per [CLI] response line, tag stripped
    is_error: bool          # True if any line matched the error pattern
    timed_out: bool         # True if no response arrived within response_timeout_s


class CliResponder:
    """Pairs outbound CLI commands with their streamed responses.

    Thread-safety: this object is driven from the asyncio event loop only.
    The serial-link thread feeds lines via ``on_ascii_line`` through a
    thread-safe queue (see serial_link.py); all other methods are async.
    """

    ERROR_MARKER = '[CLI] err:'
    RESPONSE_TAG = '[CLI] '

    def __init__(self, send_fn: SendBytesFn, *,
                 response_idle_s: float = 0.150,
                 response_timeout_s: float = 2.0):
        self._send_fn = send_fn
        self._response_idle_s = response_idle_s
        self._response_timeout_s = response_timeout_s

        # The currently in-flight command awaiting its response, if any.
        self._pending_future: Optional[asyncio.Future] = None
        self._pending_lines: list = []
        self._pending_is_error: bool = False
        self._last_line_ts: float = 0.0
        self._lock = asyncio.Lock()   # serialize senders

    # ------------------------------------------------------------------ #
    # Inbound: called by the serial link for every ASCII line decoded.
    # Must be invoked on the event loop thread.
    # ------------------------------------------------------------------ #
    def on_ascii_line(self, raw_line: str) -> None:
        """Route one decoded ASCII line.

        Lines starting with '[CLI] ' are CLI responses (paired to a command).
        Other lines ('[FOC]', '[BOOT]', '[CAN]', ...) are returned via the
        return value -- the caller (serial link) treats them as logs.
        """
        if not raw_line.startswith(self.RESPONSE_TAG):
            return None  # not a CLI response -> signal caller to log it

        # Strip the '[CLI] ' prefix; keep the rest verbatim.
        body = raw_line[len(self.RESPONSE_TAG):]
        self._pending_lines.append(body)
        self._last_line_ts = time.monotonic()
        if body.startswith('err:'):
            self._pending_is_error = True
        return None

    # ------------------------------------------------------------------ #
    # Outbound: send a command, await its response.
    # ------------------------------------------------------------------ #
    async def send(self, command: str) -> CommandResult:
        """Send a CLI command and await the streamed response.

        ``command`` must not contain a newline; the terminator is appended
        here. Raises RuntimeError if another command is in flight (should
        not happen because of the internal lock).
        """
        async with self._lock:
            if self._pending_future is not None:
                raise RuntimeError('CLI responder busy: another command is in flight')

            command = command.strip()
            if not command:
                return CommandResult(text='', lines=[], is_error=True, timed_out=True)

            self._pending_lines = []
            self._pending_is_error = False
            fut: asyncio.Future = asyncio.get_running_loop().create_future()
            self._pending_future = fut
            send_ts = time.monotonic()
            self._last_line_ts = send_ts

            # Firmware accepts \r, \n or \r\n as line terminators. Use \r\n
            # which is the most robust across host stacks.
            await self._send_fn((command + '\r\n').encode('ascii', errors='replace'))

            try:
                await self._await_response(fut, send_ts)
            finally:
                self._pending_future = None

            text = '\n'.join(self._pending_lines)
            return CommandResult(
                text=text,
                lines=list(self._pending_lines),
                is_error=self._pending_is_error,
                timed_out=fut.done() and fut.cancelled(),
            )

    async def _await_response(self, fut: asyncio.Future, send_ts: float) -> None:
        """Poll until the response stream goes idle or hard timeout fires."""
        deadline = send_ts + self._response_timeout_s
        while True:
            await asyncio.sleep(0.020)  # 20 ms poll
            now = time.monotonic()

            if self._pending_lines:
                # We have at least one line; end when the wire has been quiet.
                if (now - self._last_line_ts) >= self._response_idle_s:
                    if not fut.done():
                        fut.set_result(None)
                    return
            else:
                # No line yet; apply the hard timeout.
                if now >= deadline:
                    if not fut.done():
                        # Timed out with zero response lines.
                        self._pending_lines.append('(no response)')
                        fut.set_result(None)
                    return

    # ------------------------------------------------------------------ #
    # Best-effort fire-and-forget (e.g. for heartbeats where we don't care
    # about the response). Still serialized through the lock.
    # ------------------------------------------------------------------ #
    async def send_nowait(self, command: str) -> None:
        """Send without collecting the response. Useful for heartbeats.

        Note: any [CLI] response lines that arrive will still be consumed by
        on_ascii_line and silently discarded (since no future is waiting).
        To avoid confusing a subsequent send(), this method briefly drains.
        """
        async with self._lock:
            command = command.strip()
            if not command:
                return
            await self._send_fn((command + '\r\n').encode('ascii', errors='replace'))
            # Give the firmware a moment to reply, then drop whatever came.
            await asyncio.sleep(0.060)
            self._pending_lines = []
            self._pending_is_error = False
