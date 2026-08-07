"""Browser-facing broadcast hub.

Every connected browser tab gets its own bounded outbound queue and its own
writer task. A tab that stops draining (backgrounded, throttled, laptop asleep)
gets its stale frames dropped rather than back-pressuring the robot ingest path
— dropping a scan frame is correct behaviour, blocking the motor control loop is
not.

Topic-aware coalescing: for high-rate topics only the newest frame matters, so a
slow client that misses three scans should receive the fourth, not a backlog of
all four. Log frames are never coalesced — a dropped warning is a lost warning.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
from typing import Any

logger = logging.getLogger("slam_bot.hub")

# Topics where only the latest frame is useful — safe to replace in the queue.
COALESCING_TOPICS: frozenset[str] = frozenset({"status", "scan", "map", "tuning"})

CLIENT_QUEUE_SIZE = 64


class Client:
    """One browser WebSocket connection."""

    __slots__ = ("id", "ws", "queue", "latest", "dropped", "sent", "alive")

    def __init__(self, client_id: int, ws: Any) -> None:
        self.id = client_id
        self.ws = ws
        self.queue: asyncio.Queue[str] = asyncio.Queue(maxsize=CLIENT_QUEUE_SIZE)
        # topic -> pending payload, for coalescing topics only
        self.latest: dict[str, str] = {}
        self.dropped = 0
        self.sent = 0
        self.alive = True


class Hub:
    def __init__(self) -> None:
        self._clients: dict[int, Client] = {}
        self._next_id = 1

    # -- lifecycle ---------------------------------------------------------
    def add(self, ws: Any) -> Client:
        client = Client(self._next_id, ws)
        self._next_id += 1
        self._clients[client.id] = client
        return client

    def remove(self, client: Client) -> None:
        client.alive = False
        self._clients.pop(client.id, None)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    # -- sending -----------------------------------------------------------
    def broadcast(self, topic: str, payload: Any) -> None:
        """Queue a frame for every connected browser. Never blocks, never raises."""
        if not self._clients:
            return
        frame = json.dumps({"topic": topic, "data": payload}, separators=(",", ":"))
        for client in list(self._clients.values()):
            self._enqueue(client, topic, frame)

    def send_to(self, client: Client, topic: str, payload: Any) -> None:
        frame = json.dumps({"topic": topic, "data": payload}, separators=(",", ":"))
        self._enqueue(client, topic, frame)

    def _enqueue(self, client: Client, topic: str, frame: str) -> None:
        if not client.alive:
            return
        try:
            client.queue.put_nowait(frame)
        except asyncio.QueueFull:
            if topic in COALESCING_TOPICS:
                # Drop this client's oldest frame to make room for the newest.
                with contextlib.suppress(asyncio.QueueEmpty):
                    client.queue.get_nowait()
                    client.dropped += 1
                with contextlib.suppress(asyncio.QueueFull):
                    client.queue.put_nowait(frame)
            else:
                client.dropped += 1

    async def writer(self, client: Client) -> None:
        """Pump one client's queue to its socket until it closes."""
        try:
            while client.alive:
                frame = await client.queue.get()
                await client.ws.send_text(frame)
                client.sent += 1
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # socket closed mid-send is routine
            logger.debug("hub writer for client %s ended: %s", client.id, exc)
        finally:
            client.alive = False

    def stats(self) -> dict[str, Any]:
        return {
            "clients": len(self._clients),
            "per_client": [
                {"id": c.id, "sent": c.sent, "dropped": c.dropped}
                for c in self._clients.values()
            ],
        }


HUB = Hub()
