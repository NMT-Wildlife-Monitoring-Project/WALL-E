"""
Robot Router
Handles robot stats and navigation goal endpoints

Generated/Edited by Claude
"""

import asyncio
import logging
import time
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect, status
from pydantic import BaseModel, Field

from services.rosbridge_client import get_rosbridge_client

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["robot"])


# ============================================================================
# Models
# ============================================================================

class RobotStats(BaseModel):
    """Current robot statistics and state."""
    connected: bool = Field(description="Whether robot is connected")
    battery_voltage: float = Field(default=0.0, description="Battery voltage in volts")
    battery_percentage: float = Field(default=0.0, description="Battery percentage 0-100")
    cpu_temperature: float = Field(default=0.0, description="CPU temperature in Celsius")
    gps_fix: bool = Field(default=False, description="Whether GPS has a fix")
    gps_satellites: int = Field(default=0, description="Number of GPS satellites")
    latitude: Optional[float] = Field(default=None, description="Current latitude")
    longitude: Optional[float] = Field(default=None, description="Current longitude")
    altitude: Optional[float] = Field(default=None, description="Current altitude in meters")
    heading: float = Field(default=0.0, description="Current heading in degrees")
    speed: float = Field(default=0.0, description="Current speed in m/s")
    mode: str = Field(default="UNKNOWN", description="Current operation mode")
    armed: bool = Field(default=False, description="Whether robot is armed")
    navigation_active: bool = Field(default=False, description="Whether navigation is active")
    current_waypoint: Optional[int] = Field(default=None, description="Current waypoint index")
    distance_to_goal: Optional[float] = Field(default=None, description="Distance to current goal in meters")
    timestamp: float = Field(default_factory=time.time, description="Stats timestamp")


class NavigationGoal(BaseModel):
    """Navigation goal request."""
    lat: Optional[float] = Field(default=None, ge=-90, le=90, description="Target latitude")
    lon: Optional[float] = Field(default=None, ge=-180, le=180, description="Target longitude")
    x: Optional[float] = Field(default=None, description="Target X coordinate (local frame)")
    y: Optional[float] = Field(default=None, description="Target Y coordinate (local frame)")
    yaw_deg: Optional[float] = Field(default=None, description="Target yaw in degrees")
    speed: Optional[float] = Field(default=None, ge=0, description="Target speed in m/s")


class NavigationResponse(BaseModel):
    """Response to navigation goal request."""
    success: bool
    message: str
    goal_id: Optional[str] = None


class RobotCommand(BaseModel):
    """Generic robot command."""
    command: str = Field(..., description="Command name")
    params: Dict[str, Any] = Field(default_factory=dict, description="Command parameters")


# ============================================================================
# State management (placeholder until ROS integration)
# ============================================================================

# Placeholder robot state - will be replaced with actual ROS data
_robot_state = RobotStats(
    connected=False,
    battery_voltage=12.6,
    battery_percentage=85.0,
    cpu_temperature=45.0,
    gps_fix=False,
    gps_satellites=0,
    latitude=None,
    longitude=None,
    altitude=None,
    heading=0.0,
    speed=0.0,
    mode="IDLE",
    armed=False,
    navigation_active=False,
    current_waypoint=None,
    distance_to_goal=None,
)


def update_robot_state(**kwargs):
    """Update robot state with new values."""
    global _robot_state
    for key, value in kwargs.items():
        if hasattr(_robot_state, key):
            setattr(_robot_state, key, value)
    _robot_state.timestamp = time.time()


# ============================================================================
# REST Endpoints
# ============================================================================

@router.get("/robot/stats", response_model=RobotStats)
async def get_robot_stats():
    """
    Get current robot statistics and state.

    Returns battery, GPS, navigation status, and other telemetry data.
    This is placeholder data until ROS integration is complete.
    """
    rosbridge = get_rosbridge_client()
    _robot_state.connected = rosbridge.connected
    _robot_state.timestamp = time.time()
    return _robot_state


@router.post("/navigation/goal", response_model=NavigationResponse)
async def send_navigation_goal(goal: NavigationGoal):
    """
    Send a navigation goal to the robot.

    Accepts either GPS coordinates (lat/lon) or local coordinates (x/y).
    At least one coordinate pair must be provided.
    """
    # Validate that at least one coordinate pair is provided
    has_gps = goal.lat is not None and goal.lon is not None
    has_local = goal.x is not None and goal.y is not None

    if not has_gps and not has_local:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Must provide either GPS coordinates (lat/lon) or local coordinates (x/y)"
        )

    rosbridge = get_rosbridge_client()

    if not rosbridge.connected:
        logger.warning("ROSBridge not connected, goal will be queued")
        return NavigationResponse(
            success=False,
            message="ROSBridge not connected. Goal cannot be sent.",
            goal_id=None
        )

    try:
        # Build the navigation goal message
        if has_gps:
            # GPS-based goal - publish to GPS waypoint topic
            goal_msg = {
                "latitude": goal.lat,
                "longitude": goal.lon,
                "altitude": 0.0,
                "yaw": goal.yaw_deg if goal.yaw_deg is not None else 0.0,
            }
            topic = "/navigation/gps_goal"
            msg_type = "geographic_msgs/GeoPoseStamped"
        else:
            # Local coordinate goal - publish to move_base goal
            goal_msg = {
                "header": {
                    "frame_id": "map",
                    "stamp": {"sec": int(time.time()), "nanosec": 0}
                },
                "pose": {
                    "position": {"x": goal.x, "y": goal.y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
            topic = "/move_base_simple/goal"
            msg_type = "geometry_msgs/PoseStamped"

        success = await rosbridge.publish(topic, goal_msg, msg_type)

        if success:
            goal_id = f"goal_{int(time.time() * 1000)}"
            update_robot_state(navigation_active=True)
            logger.info(f"Navigation goal sent: {goal_msg}")
            return NavigationResponse(
                success=True,
                message=f"Navigation goal sent to {topic}",
                goal_id=goal_id
            )
        else:
            return NavigationResponse(
                success=False,
                message="Failed to publish navigation goal",
                goal_id=None
            )

    except Exception as e:
        logger.error(f"Failed to send navigation goal: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to send navigation goal: {e}"
        )


@router.post("/navigation/cancel", response_model=NavigationResponse)
async def cancel_navigation():
    """
    Cancel the current navigation goal.
    """
    rosbridge = get_rosbridge_client()

    if not rosbridge.connected:
        return NavigationResponse(
            success=False,
            message="ROSBridge not connected",
            goal_id=None
        )

    try:
        # Publish cancel message
        success = await rosbridge.publish(
            "/move_base/cancel",
            {},
            "actionlib_msgs/GoalID"
        )

        if success:
            update_robot_state(navigation_active=False, current_waypoint=None)
            logger.info("Navigation cancelled")
            return NavigationResponse(
                success=True,
                message="Navigation cancelled",
                goal_id=None
            )
        else:
            return NavigationResponse(
                success=False,
                message="Failed to send cancel command",
                goal_id=None
            )

    except Exception as e:
        logger.error(f"Failed to cancel navigation: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to cancel navigation: {e}"
        )


@router.post("/robot/command", response_model=Dict[str, Any])
async def send_robot_command(command: RobotCommand):
    """
    Send a generic command to the robot.

    Supported commands:
    - arm: Arm the robot
    - disarm: Disarm the robot
    - set_mode: Set operation mode (params: {mode: string})
    - emergency_stop: Emergency stop
    """
    rosbridge = get_rosbridge_client()

    valid_commands = ["arm", "disarm", "set_mode", "emergency_stop", "home"]

    if command.command not in valid_commands:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid command. Valid commands: {valid_commands}"
        )

    if not rosbridge.connected:
        return {
            "success": False,
            "message": "ROSBridge not connected",
            "command": command.command
        }

    try:
        # Map commands to ROS services/topics
        if command.command == "arm":
            result = await rosbridge.call_service("/mavros/cmd/arming", {"value": True})
            update_robot_state(armed=True)

        elif command.command == "disarm":
            result = await rosbridge.call_service("/mavros/cmd/arming", {"value": False})
            update_robot_state(armed=False)

        elif command.command == "set_mode":
            mode = command.params.get("mode", "MANUAL")
            result = await rosbridge.call_service(
                "/mavros/set_mode",
                {"custom_mode": mode}
            )
            update_robot_state(mode=mode)

        elif command.command == "emergency_stop":
            # Publish emergency stop
            await rosbridge.publish("/emergency_stop", {"data": True}, "std_msgs/Bool")
            update_robot_state(armed=False, navigation_active=False)
            result = {"success": True}

        elif command.command == "home":
            result = await rosbridge.call_service("/mavros/cmd/set_home", {"current_gps": True})

        logger.info(f"Robot command executed: {command.command}")
        return {
            "success": True,
            "message": f"Command '{command.command}' executed",
            "command": command.command,
            "result": result
        }

    except Exception as e:
        logger.error(f"Failed to execute command {command.command}: {e}")
        return {
            "success": False,
            "message": str(e),
            "command": command.command
        }


# ============================================================================
# WebSocket for real-time ROS data
# ============================================================================

class ConnectionManager:
    """Manages WebSocket connections for broadcasting ROS data."""

    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"WebSocket connected. Total connections: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        logger.info(f"WebSocket disconnected. Total connections: {len(self.active_connections)}")

    async def broadcast(self, message: Dict[str, Any]):
        """Broadcast message to all connected clients."""
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.error(f"Failed to send to WebSocket: {e}")


manager = ConnectionManager()


@router.websocket("/ws/ros")
async def websocket_ros_proxy(websocket: WebSocket):
    """
    WebSocket endpoint for real-time ROS data subscription.

    Clients can send subscription requests:
    {
        "action": "subscribe",
        "topic": "/robot/status",
        "type": "std_msgs/String"  // optional
    }

    {
        "action": "unsubscribe",
        "topic": "/robot/status"
    }

    The server will forward ROS messages to the client.
    """
    await manager.connect(websocket)
    rosbridge = get_rosbridge_client()

    # Track subscriptions for this connection
    subscriptions: Dict[str, bool] = {}

    async def handle_ros_message(topic: str, msg: Dict[str, Any]):
        """Forward ROS message to WebSocket client."""
        await websocket.send_json({
            "type": "ros_message",
            "topic": topic,
            "data": msg,
            "timestamp": time.time()
        })

    try:
        # Send initial connection status
        await websocket.send_json({
            "type": "connection_status",
            "rosbridge_connected": rosbridge.connected,
            "timestamp": time.time()
        })

        while True:
            # Receive message from client
            data = await websocket.receive_json()
            action = data.get("action")

            if action == "subscribe":
                topic = data.get("topic")
                msg_type = data.get("type")

                if topic and topic not in subscriptions:
                    # Create callback for this topic
                    async def topic_callback(msg, t=topic):
                        await handle_ros_message(t, msg)

                    success = await rosbridge.subscribe(topic, topic_callback, msg_type)
                    subscriptions[topic] = True

                    await websocket.send_json({
                        "type": "subscription_result",
                        "topic": topic,
                        "success": success,
                        "message": "Subscribed" if success else "Failed to subscribe"
                    })

            elif action == "unsubscribe":
                topic = data.get("topic")
                if topic and topic in subscriptions:
                    await rosbridge.unsubscribe(topic)
                    del subscriptions[topic]

                    await websocket.send_json({
                        "type": "subscription_result",
                        "topic": topic,
                        "success": True,
                        "message": "Unsubscribed"
                    })

            elif action == "publish":
                topic = data.get("topic")
                msg = data.get("msg")
                msg_type = data.get("type")

                if topic and msg:
                    success = await rosbridge.publish(topic, msg, msg_type)
                    await websocket.send_json({
                        "type": "publish_result",
                        "topic": topic,
                        "success": success
                    })

            elif action == "ping":
                await websocket.send_json({
                    "type": "pong",
                    "rosbridge_connected": rosbridge.connected,
                    "timestamp": time.time()
                })

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        # Cleanup subscriptions
        for topic in subscriptions:
            await rosbridge.unsubscribe(topic)
        manager.disconnect(websocket)
