"""
density_map.py
==============
Component 1 of D2OC Algorithm: Probabilistic Occupancy Grid (Density Map)
This module implements a 2-D occupancy grid that represents the environment as a grid of cells, each storing:
  - occupancy:  probability that a physical obstacle is present  (0.0-1.0)
  - confidence: how much evidence we have for that classification (0.0-1.0)
  - observation_count: raw number of times this cell was observed

How it is updated:
Every time a new LiDAR scan arrives, update_from_scan() is called.
It traces a straight line (ray) from the robot position to each LiDAR
range reading.  Every cell the ray passes through is FREE; the cell
where the ray ends (the obstacle hit point) is OCCUPIED.
This is known as ray-casting.

Bayesian update:
Rather than simply overwriting values, we use log-odds Bayesian updating
so the grid gets more confident with repeated observations and never locks
permanently to 0 or 1.

  log_odds(p) = log( p / (1-p) )

  update rule:
    L_new = L_old + L_sensor - L_prior

  Then convert back: p = exp(L) / (1 + exp(L))

Grid layout:
  size: 500 x 500 cells (default-> can be changed later if needed)
  resolution: 0.2 m per cell  →  covers 100 m × 100 m 
  origin: world coordinate (0,0) maps to grid centre cell (250, 250)
"""

import math
import numpy as np

# ROS2 message type for exporting the grid to RViz
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


class DensityMap:
    """
    Probabilistic 2-D occupancy grid built from LiDAR ray-casting.

    Parameters
    width      : total width  of the grid in metres  (default 100 m)
    height     : total height of the grid in metres  (default 100 m)
    resolution : side length of each cell in metres  (default 0.1 m)
    """

    # ------------------------------------------------------------------ #
    #  Log-odds constants for the Bayesian sensor model                    #
    # ------------------------------------------------------------------ #
    # Sensor reading "this cell is FREE"  → low probability of obstacle
    _LOG_ODDS_FREE     = math.log(0.1 / 0.9)   # ≈ -2.197

    # Sensor reading "this cell is OCCUPIED" → high probability of obstacle
    _LOG_ODDS_OCCUPIED = math.log(0.9 / 0.1)   # ≈  2.197

    # Prior belief before any observation: completely unknown (p = 0.5)
    _LOG_ODDS_PRIOR    = math.log(0.5 / 0.5)   # = 0.0

    # Hard clamp so log-odds never grows without bound
    _LOG_ODDS_MAX =  10.0
    _LOG_ODDS_MIN = -10.0

    # Maximum confidence value a single cell can reach
    _CONFIDENCE_MAX_OBSERVATIONS = 50  # observations until confidence = 1.0

    def __init__(self, width: float = 100.0, height: float = 100.0,
                 resolution: float = 0.1):
        """
        Initialise the grid arrays and store configuration.

        After __init__ the grid represents a completely unknown environment:
          occupancy         = 0.5  everywhere  (50 % chance of obstacle)
          confidence        = 0.0  everywhere  (zero evidence)
          observation_count = 0    everywhere
          log_odds          = 0.0  everywhere  (matches prior = 0.5)
        """
        self.resolution = resolution          # metres per cell
        self.width_m    = width               # total width  in metres
        self.height_m   = height              # total height in metres

        # Number of cells along each axis
        self.cols = int(width  / resolution)  # x-axis  (e.g. 500)
        self.rows = int(height / resolution)  # y-axis  (e.g. 500)

        # The world coordinate that corresponds to grid cell (0, 0).
        # We place the origin at the centre of the grid so the robot
        # can map 50 m in every direction from its starting location.
        self.origin_x = -width  / 2.0        # e.g. -50.0 m
        self.origin_y = -height / 2.0        # e.g. -50.0 m

        # ---- Core data arrays ---------------------------------------- #

        # log-odds representation used for Bayesian updates
        # Initialised to 0.0  ↔  p = 0.5 (unknown)
        self._log_odds = np.zeros((self.rows, self.cols), dtype=np.float32)

        # Public-facing occupancy probability derived from _log_odds
        # Shape: (rows, cols)  Values: [0.0, 1.0]
        self.occupancy = np.full((self.rows, self.cols), 0.5, dtype=np.float32)

        # How certain we are about each cell's classification
        # Shape: (rows, cols)  Values: [0.0, 1.0]
        self.confidence = np.zeros((self.rows, self.cols), dtype=np.float32)

        # Raw count of how many times each cell was observed by the sensor
        # Shape: (rows, cols)  Values: non-negative integers
        self.observation_count = np.zeros((self.rows, self.cols), dtype=np.int32)

        # Frame id stored so to_occupancy_grid() can fill the ROS header
        self.frame_id = 'map'

    # ------------------------------------------------------------------ #
    #  Coordinate conversion helpers                                       #
    # ------------------------------------------------------------------ #

    def world_to_grid(self, world_x: float, world_y: float):
        """
        Convert a world-frame position (metres) to grid indices (col, row).

        The world origin (0, 0) maps to the centre of the grid.

        Returns
        -------
        (col, row) as integers, or None if the point is outside the grid.

        Example
        -------
        world (0, 0)  →  grid (250, 250)   [with 500×500 default grid]
        world (10, 5) →  grid (300, 275)   [10/0.2 = 50 cols right of centre]
        """
        col = int((world_x - self.origin_x) / self.resolution)
        row = int((world_y - self.origin_y) / self.resolution)

        if 0 <= col < self.cols and 0 <= row < self.rows:
            return col, row
        return None   # outside the grid boundary

    def grid_to_world(self, col: int, row: int):
        """
        Convert grid indices (col, row) to the world-frame centre of that cell.

        Returns
        -------
        (world_x, world_y) in metres.

        Example
        -------
        grid (250, 250) → world (0.1, 0.1)   [centre of cell, not exact 0,0]
        """
        world_x = self.origin_x + (col + 0.5) * self.resolution
        world_y = self.origin_y + (row + 0.5) * self.resolution
        return world_x, world_y

    def is_in_bounds(self, col: int, row: int) -> bool:
        """Return True if (col, row) is a valid grid index."""
        return 0 <= col < self.cols and 0 <= row < self.rows

    # ------------------------------------------------------------------ #
    #  Main update: process one LiDAR scan                                 #
    # ------------------------------------------------------------------ #

    def update_from_scan(self, scan_msg, robot_x: float, robot_y: float,
                         robot_theta: float, confidence: float = 0.9):
        """
        Update the density map using a single LaserScan message.

        For every valid range reading the method:
          1. Calculates the absolute angle of the ray in the world frame.
          2. Computes the (x, y) hit point in the world frame.
          3. Uses Bresenham ray-casting to walk every cell between the robot
             and the hit point → marks those cells FREE.
          4. Marks the hit-point cell OCCUPIED.
          5. Applies Bayesian log-odds update to each visited cell.

        Parameters
        ----------
        scan_msg    : sensor_msgs/LaserScan   — incoming LiDAR message
        robot_x     : robot position x  in the world/map frame  (metres)
        robot_y     : robot position y  in the world/map frame  (metres)
        robot_theta : robot heading     in the world/map frame  (radians)
        confidence  : sensor reliability weight  [0.0, 1.0]
                      default 0.9 because LiDAR is very reliable
        """
        angle      = scan_msg.angle_min          # current ray angle (robot frame)
        angle_inc  = scan_msg.angle_increment
        range_min  = scan_msg.range_min
        range_max  = scan_msg.range_max

        # Get the robot's grid cell once – used as the ray start
        robot_cell = self.world_to_grid(robot_x, robot_y)
        if robot_cell is None:
            return   # robot is outside the mapped area – nothing to do

        robot_col, robot_row = robot_cell

        for r in scan_msg.ranges:

            # ----------------------------------------------------------
            # Handle invalid and max-range readings
            # ----------------------------------------------------------
            if math.isnan(r) or r < range_min:
                angle += angle_inc
                continue

            # LiDAR no-return (inf) or clipped values beyond max range should
            # still contribute free-space evidence up to range_max.
            hit_is_obstacle = True
            if math.isinf(r) or r > range_max:
                r = range_max
                hit_is_obstacle = False

            # ----------------------------------------------------------
            # Step 1: angle of this ray in the WORLD frame
            # robot_theta rotates the robot-frame ray into world frame
            # ----------------------------------------------------------
            world_angle = robot_theta + angle

            # ----------------------------------------------------------
            # Step 2: hit point coordinates in the world frame
            # ----------------------------------------------------------
            hit_x = robot_x + r * math.cos(world_angle)
            hit_y = robot_y + r * math.sin(world_angle)

            hit_cell = self.world_to_grid(hit_x, hit_y)

            # ----------------------------------------------------------
            # Step 3 & 4: ray-cast FREE, then mark end cell OCCUPIED
            # ----------------------------------------------------------
            if hit_cell is not None:
                hit_col, hit_row = hit_cell

                # Walk every cell from robot to the cell BEFORE the hit point
                free_cells = self._bresenham(robot_col, robot_row,
                                             hit_col,   hit_row)

                # All traversed cells (except the last) are free space
                for (c, rw) in free_cells[:-1]:
                    self._bayesian_update(c, rw, occupied=False,
                                          confidence=confidence)

                # The final cell is occupied only when we truly hit an obstacle.
                if hit_is_obstacle:
                    self._bayesian_update(hit_col, hit_row, occupied=True,
                                          confidence=confidence)
                else:
                    self._bayesian_update(hit_col, hit_row, occupied=False,
                                          confidence=confidence)
            else:
                # Hit point is outside the grid – still mark free cells
                free_cells = self._bresenham_to_edge(
                    robot_col, robot_row, world_angle, range_max)
                for (c, rw) in free_cells:
                    self._bayesian_update(c, rw, occupied=False,
                                          confidence=confidence)

            angle += angle_inc

    # ------------------------------------------------------------------ #
    #  Bayesian log-odds update                                            #
    # ------------------------------------------------------------------ #

    def _bayesian_update(self, col: int, row: int, occupied: bool,
                          confidence: float):
        """
        Update one cell using the Bayesian log-odds rule.

        Log-odds update formula:
            L_new = L_old + L_sensor - L_prior

        Where:
            L_sensor = log_odds_occupied  if occupied  (≈ +2.2)
                     = log_odds_free      if free      (≈ -2.2)
            L_prior  = 0.0  (our starting belief is p=0.5)

        The confidence weight scales how strongly each observation
        shifts the log-odds value so a more reliable sensor (e.g. LiDAR
        at 0.9) updates the map faster than a noisy sensor would.

        After the update the occupancy probability and confidence are
        derived from the new log-odds value.
        """
        if not self.is_in_bounds(col, row):
            return

        # Choose the log-odds sensor value based on what was observed
        if occupied:
            l_sensor = self._LOG_ODDS_OCCUPIED
        else:
            l_sensor = self._LOG_ODDS_FREE

        # Apply the update, scaled by confidence, subtract prior (= 0)
        self._log_odds[row, col] += confidence * (l_sensor - self._LOG_ODDS_PRIOR)

        # Clamp so values never diverge to ±∞ (prevents lock-in)
        self._log_odds[row, col] = np.clip(
            self._log_odds[row, col], self._LOG_ODDS_MIN, self._LOG_ODDS_MAX)

        # Convert log-odds back to probability:  p = e^L / (1 + e^L)
        l = self._log_odds[row, col]
        self.occupancy[row, col] = float(np.exp(l) / (1.0 + np.exp(l)))

        # Increment raw observation count
        self.observation_count[row, col] += 1

        # Confidence grows with more observations, saturating at 1.0
        # Uses a simple ratio capped at _CONFIDENCE_MAX_OBSERVATIONS
        obs = self.observation_count[row, col]
        self.confidence[row, col] = min(
            1.0, obs / self._CONFIDENCE_MAX_OBSERVATIONS)

    # ------------------------------------------------------------------ #
    #  Ray-casting helpers (Bresenham's line algorithm)                    #
    # ------------------------------------------------------------------ #

    def _bresenham(self, c0: int, r0: int, c1: int, r1: int):
        """
        Return a list of (col, row) grid cells on the line from
        (c0, r0) to (c1, r1) inclusive, using Bresenham's algorithm.

        Bresenham's line algorithm is the standard efficient integer-only
        method for enumerating all grid cells that a straight line passes
        through – exactly what we need for ray-casting.
        """
        cells = []
        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        err = dc - dr

        c, r = c0, r0
        while True:
            if self.is_in_bounds(c, r):
                cells.append((c, r))
            if c == c1 and r == r1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c   += sc
            if e2 <  dc:
                err += dc
                r   += sr
        return cells

    def _bresenham_to_edge(self, start_col: int, start_row: int,
                           angle: float, max_range: float):
        """
        Walk cells along `angle` from (start_col, start_row) until we reach
        the grid boundary or exceed max_range. Used when the hit point is
        outside the grid – we still want to mark the free cells inside.

        Returns list of (col, row) cells within the grid.
        """
        end_x = (self.origin_x + start_col * self.resolution
                 + max_range * math.cos(angle))
        end_y = (self.origin_y + start_row * self.resolution
                 + max_range * math.sin(angle))

        end_col = int((end_x - self.origin_x) / self.resolution)
        end_row = int((end_y - self.origin_y) / self.resolution)

        # Clamp to grid boundaries
        end_col = max(0, min(self.cols - 1, end_col))
        end_row = max(0, min(self.rows - 1, end_row))

        return self._bresenham(start_col, start_row, end_col, end_row)

    # ------------------------------------------------------------------ #
    #  Public accessors                                                    #
    # ------------------------------------------------------------------ #

    def get_occupancy(self, col: int, row: int) -> float:
        """
        Return the occupancy probability at grid cell (col, row).

        Returns
        -------
        float in [0.0, 1.0]
          0.0 → certainly free space
          0.5 → completely unknown (never observed)
          1.0 → certainly occupied
        Returns 0.5 (unknown) if the cell is outside the grid.
        """
        if not self.is_in_bounds(col, row):
            return 0.5
        return float(self.occupancy[row, col])

    def get_confidence(self, col: int, row: int) -> float:
        """
        Return how certain we are about the classification of (col, row).

        Returns
        -------
        float in [0.0, 1.0]
          0.0 → never observed, no evidence
          1.0 → many observations, high certainty
        Returns 0.0 if the cell is outside the grid.
        """
        if not self.is_in_bounds(col, row):
            return 0.0
        return float(self.confidence[row, col])

    def get_observation_count(self, col: int, row: int) -> int:
        """Return how many times cell (col, row) has been observed."""
        if not self.is_in_bounds(col, row):
            return 0
        return int(self.observation_count[row, col])

    def reset(self):
        """
        Reset the entire grid back to the initial unknown state.
        Useful for restarting exploration in a new environment.
        """
        self._log_odds[::]        = 0.0
        self.occupancy[::]        = 0.5
        self.confidence[::]       = 0.0
        self.observation_count[::] = 0

    # ------------------------------------------------------------------ #
    #  ROS2 export for RViz visualisation                                  #
    # ------------------------------------------------------------------ #

    def to_occupancy_grid(self, stamp=None, frame_id: str = 'map') -> OccupancyGrid:
        """
        Convert the internal density map into a ROS2 OccupancyGrid message
        so it can be published and visualised in RViz.

        ROS OccupancyGrid uses int8 values in the range [-1, 100]:
          -1   → unknown  (we map occupancy = 0.5 here)
           0   → free     (occupancy close to 0.0)
          100  → occupied (occupancy close to 1.0)

        The data array is row-major, starting from the bottom-left corner
        of the map (origin corner), so we flip the rows.

        Parameters
        ----------
        stamp    : builtin_interfaces/Time  — optional ROS timestamp
        frame_id : coordinate frame string  — default 'map'

        Returns
        -------
        nav_msgs/OccupancyGrid ready to publish.
        """
        msg = OccupancyGrid()

        # --- Header ---------------------------------------------------
        msg.header = Header()
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp

        # --- Map metadata ---------------------------------------------
        msg.info.resolution  = self.resolution
        msg.info.width       = self.cols
        msg.info.height      = self.rows

        # Origin pose: position of cell (0,0) in the world frame
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # no rotation

        # --- Data conversion ------------------------------------------
        # occupancy array is (rows, cols) float32 in [0.0, 1.0]
        # ROS wants a flat list of int8 in [-1, 100], row-major

        data = np.full((self.rows, self.cols), -1, dtype=np.int8)  # default: unknown

        # Where confidence is high enough, convert the occupancy probability
        known_mask = self.confidence > 0.05   # at least some observations

        # Map [0.0, 1.0] → [0, 100] for known cells
        data[known_mask] = (self.occupancy[known_mask] * 100).astype(np.int8)

        # Flatten row-major (ROS expects row 0 = bottom of map)
        msg.data = data.flatten().tolist()

        return msg
