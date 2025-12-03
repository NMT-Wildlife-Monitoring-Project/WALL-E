"""
ROSBridge WebSocket Client Service
Handles communication with ROS via rosbridge_server

Generated/Edited by Claude
"""

import asyncio
import json
import logging
from typing import Any, Callable, Dict, Optional
from contextlib import asynccontextmanager

import websockets
from websockets.exceptions import ConnectionClosed, WebSocketException

logger = logging.getLogger(__name__)


class ROSBridgeClient:
    """
    Async WebSocket client for ROSBridge communication.
    Manages subscriptions, publications, and service calls to ROS.
    """

    def __init__(self, uri: str = "ws://localhost:9090"):
        self.uri = uri
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self._connected = False
        self._subscribers: Dict[str, Callable] = {}
        self._message_id = 0
        self._pending_calls: Dict[str, asyncio.Future] = {}
        self._receive_task: Optional[asyncio.Task] = None
        self._reconnect_task: Optional[asyncio.Task] = None
        self._should_reconnect = True
        self._lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        return self._connected and self.websocket is not None

    def _get_message_id(self) -> str:
        self._message_id += 1
        return f"msg_{self._message_id}"

    async def connect(self) -> bool:
        """Establish connection to ROSBridge server."""
        async with self._lock:
            if self._connected:
                return True

            try:
                self.websocket = await asyncio.wait_for(
                    websockets.connect(self.uri),
                    timeout=5.0
                )
                self._connected = True
                self._receive_task = asyncio.create_task(self._receive_loop())
                logger.info(f"Connected to ROSBridge at {self.uri}")
                return True
            except asyncio.TimeoutError:
                logger.warning(f"Connection to ROSBridge timed out: {self.uri}")
                return False
            except Exception as e:
                logger.warning(f"Failed to connect to ROSBridge: {e}")
                return False

    async def disconnect(self):
        """Close connection to ROSBridge server."""
        self._should_reconnect = False
        async with self._lock:
            self._connected = False

            if self._receive_task:
                self._receive_task.cancel()
                try:
                    await self._receive_task
                except asyncio.CancelledError:
                    pass
                self._receive_task = None

            if self._reconnect_task:
                self._reconnect_task.cancel()
                try:
                    await self._reconnect_task
                except asyncio.CancelledError:
                    pass
                self._reconnect_task = None

            if self.websocket:
                await self.websocket.close()
                self.websocket = None

            self._subscribers.clear()
            self._pending_calls.clear()
            logger.info("Disconnected from ROSBridge")

    async def _receive_loop(self):
        """Background task to receive and dispatch messages."""
        while self._connected and self.websocket:
            try:
                message = await self.websocket.recv()
                data = json.loads(message)
                await self._handle_message(data)
            except ConnectionClosed:
                logger.warning("ROSBridge connection closed")
                self._connected = False
                if self._should_reconnect:
                    self._reconnect_task = asyncio.create_task(self._auto_reconnect())
                break
            except json.JSONDecodeError as e:
                logger.error(f"Invalid JSON from ROSBridge: {e}")
            except Exception as e:
                logger.error(f"Error in receive loop: {e}")

    async def _auto_reconnect(self):
        """Attempt to reconnect with exponential backoff."""
        delay = 1.0
        max_delay = 30.0

        while self._should_reconnect and not self._connected:
            logger.info(f"Attempting to reconnect in {delay}s...")
            await asyncio.sleep(delay)

            if await self.connect():
                # Re-subscribe to all topics
                for topic, callback in list(self._subscribers.items()):
                    await self._send_subscribe(topic)
                break

            delay = min(delay * 2, max_delay)

    async def _handle_message(self, data: Dict[str, Any]):
        """Handle incoming ROSBridge message."""
        op = data.get("op")

        if op == "publish":
            # Topic subscription message
            topic = data.get("topic")
            msg = data.get("msg")
            if topic in self._subscribers:
                try:
                    callback = self._subscribers[topic]
                    if asyncio.iscoroutinefunction(callback):
                        await callback(msg)
                    else:
                        callback(msg)
                except Exception as e:
                    logger.error(f"Error in subscriber callback for {topic}: {e}")

        elif op == "service_response":
            # Service call response
            call_id = data.get("id")
            if call_id in self._pending_calls:
                future = self._pending_calls.pop(call_id)
                if data.get("result"):
                    future.set_result(data.get("values"))
                else:
                    future.set_exception(Exception(f"Service call failed: {data}"))

        elif op == "call_service":
            # Incoming service call (if we're providing services)
            logger.debug(f"Received service call: {data}")

    async def _send(self, message: Dict[str, Any]) -> bool:
        """Send a message to ROSBridge."""
        if not self.connected or not self.websocket:
            logger.warning("Cannot send: not connected to ROSBridge")
            return False

        try:
            await self.websocket.send(json.dumps(message))
            return True
        except WebSocketException as e:
            logger.error(f"Failed to send message: {e}")
            self._connected = False
            return False

    async def _send_subscribe(self, topic: str, msg_type: Optional[str] = None) -> bool:
        """Send subscribe message to ROSBridge."""
        message = {
            "op": "subscribe",
            "topic": topic,
        }
        if msg_type:
            message["type"] = msg_type
        return await self._send(message)

    async def subscribe(
        self,
        topic: str,
        callback: Callable[[Dict[str, Any]], Any],
        msg_type: Optional[str] = None
    ) -> bool:
        """
        Subscribe to a ROS topic.

        Args:
            topic: ROS topic name (e.g., "/robot/status")
            callback: Function to call when message received
            msg_type: Optional ROS message type

        Returns:
            True if subscription successful
        """
        self._subscribers[topic] = callback

        if not self.connected:
            logger.info(f"Queued subscription to {topic} (not connected)")
            return True

        return await self._send_subscribe(topic, msg_type)

    async def unsubscribe(self, topic: str) -> bool:
        """Unsubscribe from a ROS topic."""
        self._subscribers.pop(topic, None)

        if not self.connected:
            return True

        message = {
            "op": "unsubscribe",
            "topic": topic,
        }
        return await self._send(message)

    async def publish(
        self,
        topic: str,
        msg: Dict[str, Any],
        msg_type: Optional[str] = None
    ) -> bool:
        """
        Publish a message to a ROS topic.

        Args:
            topic: ROS topic name
            msg: Message data as dictionary
            msg_type: Optional ROS message type

        Returns:
            True if publish successful
        """
        message = {
            "op": "publish",
            "topic": topic,
            "msg": msg,
        }
        if msg_type:
            message["type"] = msg_type

        return await self._send(message)

    async def call_service(
        self,
        service: str,
        args: Optional[Dict[str, Any]] = None,
        timeout: float = 10.0
    ) -> Optional[Dict[str, Any]]:
        """
        Call a ROS service.

        Args:
            service: Service name (e.g., "/set_mode")
            args: Service arguments
            timeout: Timeout in seconds

        Returns:
            Service response or None if failed
        """
        if not self.connected:
            logger.warning("Cannot call service: not connected")
            return None

        call_id = self._get_message_id()
        future: asyncio.Future = asyncio.get_event_loop().create_future()
        self._pending_calls[call_id] = future

        message = {
            "op": "call_service",
            "id": call_id,
            "service": service,
            "args": args or {},
        }

        if not await self._send(message):
            self._pending_calls.pop(call_id, None)
            return None

        try:
            result = await asyncio.wait_for(future, timeout=timeout)
            return result
        except asyncio.TimeoutError:
            self._pending_calls.pop(call_id, None)
            logger.error(f"Service call to {service} timed out")
            return None
        except Exception as e:
            logger.error(f"Service call to {service} failed: {e}")
            return None

    async def advertise(self, topic: str, msg_type: str) -> bool:
        """Advertise a topic for publishing."""
        message = {
            "op": "advertise",
            "topic": topic,
            "type": msg_type,
        }
        return await self._send(message)

    async def unadvertise(self, topic: str) -> bool:
        """Stop advertising a topic."""
        message = {
            "op": "unadvertise",
            "topic": topic,
        }
        return await self._send(message)


# Global client instance
_rosbridge_client: Optional[ROSBridgeClient] = None


def get_rosbridge_client() -> ROSBridgeClient:
    """Get or create the global ROSBridge client instance."""
    global _rosbridge_client
    if _rosbridge_client is None:
        _rosbridge_client = ROSBridgeClient()
    return _rosbridge_client


async def init_rosbridge_client(uri: str = "ws://localhost:9090") -> ROSBridgeClient:
    """Initialize and connect the ROSBridge client."""
    global _rosbridge_client
    _rosbridge_client = ROSBridgeClient(uri)
    await _rosbridge_client.connect()
    return _rosbridge_client


async def shutdown_rosbridge_client():
    """Shutdown the ROSBridge client."""
    global _rosbridge_client
    if _rosbridge_client:
        await _rosbridge_client.disconnect()
        _rosbridge_client = None
