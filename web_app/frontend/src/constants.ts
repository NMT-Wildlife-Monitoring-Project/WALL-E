/**
 * Centralized Constants for WALL-E Web Control Panel
 * Generated/Edited by Claude
 *
 * All magic numbers and configuration values should be defined here
 * to improve maintainability and consistency across the application.
 */

// =============================================================================
// Timing Constants (milliseconds)
// =============================================================================
export const TIMING = {
  /** Retry delay for camera connection attempts */
  CAMERA_RETRY_DELAY_MS: 2000,
  /** Maximum camera connection retry attempts */
  CAMERA_MAX_RETRIES: 5,
  /** Velocity command throttle interval */
  VELOCITY_THROTTLE_MS: 100,
  /** Default joystick publish rate */
  JOYSTICK_PUBLISH_RATE_MS: 100,
  /** Stats panel update debounce */
  STATS_UPDATE_DEBOUNCE_MS: 100,
  /** WebSocket reconnect initial delay */
  WS_RECONNECT_INITIAL_MS: 1000,
  /** WebSocket reconnect max delay */
  WS_RECONNECT_MAX_MS: 30000,
  /** API request timeout */
  API_TIMEOUT_MS: 10000,
} as const;

// =============================================================================
// Robot Control Constants
// =============================================================================
export const ROBOT_CONTROL = {
  /** Default max linear speed (m/s) */
  MAX_LINEAR_SPEED: 1.0,
  /** Default max angular speed (rad/s) */
  MAX_ANGULAR_SPEED: 1.5,
  /** Joystick deadzone threshold (0-1) */
  JOYSTICK_DEADZONE: 0.1,
} as const;

// =============================================================================
// Stats Thresholds for Status Coloring
// =============================================================================
export const THRESHOLDS = {
  temperature: { warning: 60, danger: 75 },
  battery: { warning: 30, danger: 15 },
  solar: { warning: 5, danger: 0 },
  cpu: { warning: 70, danger: 90 },
  memory: { warning: 70, danger: 90 },
} as const;

// =============================================================================
// Camera Configuration
// =============================================================================
export const CAMERA = {
  /** Default frame width */
  DEFAULT_WIDTH: 640,
  /** Default frame height */
  DEFAULT_HEIGHT: 480,
  /** Default FPS */
  DEFAULT_FPS: 30,
  /** Default JPEG quality (1-100) */
  DEFAULT_JPEG_QUALITY: 80,
} as const;

// =============================================================================
// Map Configuration
// =============================================================================
export const MAP = {
  /** Default map center (San Francisco) */
  DEFAULT_CENTER: { lat: 37.7749, lng: -122.4194 },
  /** Default zoom level */
  DEFAULT_ZOOM: 18,
  /** Robot marker color */
  ROBOT_MARKER_COLOR: '#3fb950',
  /** Waypoint marker color */
  WAYPOINT_MARKER_COLOR: '#58a6ff',
  /** Path polyline color */
  PATH_COLOR: '#58a6ff',
  /** Path opacity */
  PATH_OPACITY: 0.8,
  /** Path stroke weight */
  PATH_STROKE_WEIGHT: 3,
} as const;

// =============================================================================
// UI Constants
// =============================================================================
export const UI = {
  /** Joystick size in pixels */
  JOYSTICK_SIZE: 150,
  /** Joystick rest opacity */
  JOYSTICK_REST_OPACITY: 0.5,
  /** Joystick fade time */
  JOYSTICK_FADE_TIME_MS: 100,
  /** Progress bar width */
  PROGRESS_BAR_WIDTH: 60,
} as const;

// =============================================================================
// ROS Topics
// =============================================================================
export const ROS_TOPICS = {
  CMD_VEL: '/cmd_vel',
  ROBOT_STATS: '/robot/stats',
  GPS_FIX: '/fix',
  GLOBAL_COSTMAP: '/global_costmap/costmap',
  LOCAL_COSTMAP: '/local_costmap/costmap',
  ODOM: '/odom',
  SCAN: '/scan',
  WAYPOINTS: '/navigation/waypoints',
} as const;

// =============================================================================
// API Endpoints
// =============================================================================
export const API_ENDPOINTS = {
  CAMERA_STREAM: '/api/camera/stream',
  CAMERA_SNAPSHOT: '/api/camera/snapshot',
  CAMERA_STATUS: '/api/camera/status',
  ROBOT_STATS: '/api/robot/stats',
  NAVIGATION_GOAL: '/api/navigation/goal',
  NAVIGATION_CANCEL: '/api/navigation/cancel',
  WAYPOINTS: '/api/waypoints',
  HEALTH: '/health',
} as const;
