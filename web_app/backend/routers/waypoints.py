"""
Waypoints Router
Handles CRUD operations for GPS waypoints stored in YAML format

Generated/Edited by Claude
"""

import logging
import os
from pathlib import Path
from typing import List, Optional

import yaml
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field, field_validator

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/waypoints", tags=["waypoints"])

# Waypoint file path
WAYPOINTS_FILE = Path("/tmp/shared/waypoints.yaml")


class Waypoint(BaseModel):
    """Single waypoint with GPS coordinates and orientation."""
    lat: float = Field(..., description="Latitude in degrees", ge=-90, le=90)
    lon: float = Field(..., description="Longitude in degrees", ge=-180, le=180)
    alt: float = Field(default=0.0, description="Altitude in meters")
    yaw_deg: float = Field(default=0.0, description="Yaw angle in degrees", ge=0, lt=360)
    name: Optional[str] = Field(default=None, description="Optional waypoint name")

    @field_validator("yaw_deg", mode="before")
    @classmethod
    def normalize_yaw(cls, v: float) -> float:
        """Normalize yaw to 0-360 range."""
        if v is None:
            return 0.0
        return v % 360


class WaypointList(BaseModel):
    """List of waypoints matching YAML structure."""
    waypoints: List[Waypoint] = Field(default_factory=list)


class WaypointCreate(BaseModel):
    """Request body for creating a waypoint."""
    lat: float = Field(..., ge=-90, le=90)
    lon: float = Field(..., ge=-180, le=180)
    alt: float = Field(default=0.0)
    yaw_deg: float = Field(default=0.0, ge=0, lt=360)
    name: Optional[str] = None


class WaypointUpdate(BaseModel):
    """Request body for updating a waypoint."""
    lat: Optional[float] = Field(default=None, ge=-90, le=90)
    lon: Optional[float] = Field(default=None, ge=-180, le=180)
    alt: Optional[float] = None
    yaw_deg: Optional[float] = Field(default=None, ge=0, lt=360)
    name: Optional[str] = None


def ensure_waypoints_dir():
    """Ensure the waypoints directory exists."""
    WAYPOINTS_FILE.parent.mkdir(parents=True, exist_ok=True)


def load_waypoints() -> WaypointList:
    """Load waypoints from YAML file."""
    ensure_waypoints_dir()

    if not WAYPOINTS_FILE.exists():
        return WaypointList(waypoints=[])

    try:
        with open(WAYPOINTS_FILE, "r") as f:
            data = yaml.safe_load(f)

        if data is None:
            return WaypointList(waypoints=[])

        # Handle both formats: list or dict with 'waypoints' key
        if isinstance(data, list):
            waypoints = [Waypoint(**wp) for wp in data]
        elif isinstance(data, dict) and "waypoints" in data:
            waypoints = [Waypoint(**wp) for wp in data["waypoints"]]
        else:
            logger.warning(f"Unexpected waypoints format: {type(data)}")
            return WaypointList(waypoints=[])

        return WaypointList(waypoints=waypoints)

    except yaml.YAMLError as e:
        logger.error(f"Failed to parse waypoints YAML: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to parse waypoints file: {e}"
        )
    except Exception as e:
        logger.error(f"Failed to load waypoints: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to load waypoints: {e}"
        )


def save_waypoints(waypoint_list: WaypointList):
    """Save waypoints to YAML file."""
    ensure_waypoints_dir()

    try:
        # Convert to dict format for YAML
        data = {
            "waypoints": [
                {
                    "lat": wp.lat,
                    "lon": wp.lon,
                    "alt": wp.alt,
                    "yaw_deg": wp.yaw_deg,
                    **({"name": wp.name} if wp.name else {})
                }
                for wp in waypoint_list.waypoints
            ]
        }

        with open(WAYPOINTS_FILE, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        logger.info(f"Saved {len(waypoint_list.waypoints)} waypoints to {WAYPOINTS_FILE}")

    except Exception as e:
        logger.error(f"Failed to save waypoints: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to save waypoints: {e}"
        )


@router.get("", response_model=WaypointList)
async def get_waypoints():
    """
    Get all waypoints from the YAML file.

    Returns the complete list of waypoints with their GPS coordinates
    and orientation data.
    """
    return load_waypoints()


@router.post("", response_model=WaypointList, status_code=status.HTTP_201_CREATED)
async def create_waypoint(waypoint: WaypointCreate):
    """
    Add a new waypoint to the list.

    The waypoint will be appended to the existing waypoints file.
    Returns the updated list of all waypoints.
    """
    waypoint_list = load_waypoints()

    new_waypoint = Waypoint(
        lat=waypoint.lat,
        lon=waypoint.lon,
        alt=waypoint.alt,
        yaw_deg=waypoint.yaw_deg,
        name=waypoint.name
    )

    waypoint_list.waypoints.append(new_waypoint)
    save_waypoints(waypoint_list)

    logger.info(f"Created waypoint at ({waypoint.lat}, {waypoint.lon})")
    return waypoint_list


@router.put("", response_model=WaypointList)
async def replace_waypoints(waypoint_list: WaypointList):
    """
    Replace all waypoints with a new list.

    This completely overwrites the existing waypoints file.
    Use with caution.
    """
    save_waypoints(waypoint_list)
    logger.info(f"Replaced all waypoints with {len(waypoint_list.waypoints)} new waypoints")
    return waypoint_list


@router.get("/{index}", response_model=Waypoint)
async def get_waypoint(index: int):
    """
    Get a specific waypoint by index.

    Index is 0-based.
    """
    waypoint_list = load_waypoints()

    if index < 0 or index >= len(waypoint_list.waypoints):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Waypoint index {index} not found. Valid range: 0-{len(waypoint_list.waypoints) - 1}"
        )

    return waypoint_list.waypoints[index]


@router.put("/{index}", response_model=Waypoint)
async def update_waypoint(index: int, waypoint_update: WaypointUpdate):
    """
    Update a specific waypoint by index.

    Only provided fields will be updated.
    Index is 0-based.
    """
    waypoint_list = load_waypoints()

    if index < 0 or index >= len(waypoint_list.waypoints):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Waypoint index {index} not found. Valid range: 0-{len(waypoint_list.waypoints) - 1}"
        )

    current = waypoint_list.waypoints[index]

    # Update only provided fields
    update_data = waypoint_update.model_dump(exclude_unset=True)
    updated_waypoint = current.model_copy(update=update_data)

    waypoint_list.waypoints[index] = updated_waypoint
    save_waypoints(waypoint_list)

    logger.info(f"Updated waypoint {index}")
    return updated_waypoint


@router.delete("/{index}", response_model=WaypointList)
async def delete_waypoint(index: int):
    """
    Delete a specific waypoint by index.

    Index is 0-based. Returns the updated list after deletion.
    """
    waypoint_list = load_waypoints()

    if index < 0 or index >= len(waypoint_list.waypoints):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Waypoint index {index} not found. Valid range: 0-{len(waypoint_list.waypoints) - 1}"
        )

    deleted = waypoint_list.waypoints.pop(index)
    save_waypoints(waypoint_list)

    logger.info(f"Deleted waypoint {index} at ({deleted.lat}, {deleted.lon})")
    return waypoint_list


@router.delete("", response_model=WaypointList)
async def clear_waypoints():
    """
    Delete all waypoints.

    Use with caution - this cannot be undone.
    """
    waypoint_list = WaypointList(waypoints=[])
    save_waypoints(waypoint_list)

    logger.info("Cleared all waypoints")
    return waypoint_list


@router.post("/reorder", response_model=WaypointList)
async def reorder_waypoints(new_order: List[int]):
    """
    Reorder waypoints according to the provided index list.

    Args:
        new_order: List of current indices in the desired new order.
                   Example: [2, 0, 1] moves waypoint 2 to first, 0 to second, 1 to third.

    The list must contain each index exactly once.
    """
    waypoint_list = load_waypoints()
    num_waypoints = len(waypoint_list.waypoints)

    # Validate new_order
    if len(new_order) != num_waypoints:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"new_order must contain exactly {num_waypoints} indices"
        )

    if set(new_order) != set(range(num_waypoints)):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="new_order must contain each index from 0 to n-1 exactly once"
        )

    # Reorder waypoints
    reordered = [waypoint_list.waypoints[i] for i in new_order]
    waypoint_list.waypoints = reordered
    save_waypoints(waypoint_list)

    logger.info(f"Reordered waypoints: {new_order}")
    return waypoint_list
