/**
 * ROSBridge WebSocket Service with Zustand State Management
 * Generated/Edited by Claude
 */
import React from 'react';
import { create } from 'zustand';
import ROSLIB from 'roslib';

// Type definitions for ROS messages
export interface RobotStats {
  temperature: number;
  batteryLevel: number;
  solarPower: number;
  cpuUsage: number;
  memoryUsage: number;
  gpsLat: number;
  gpsLon: number;
  heading: number;
  speed: number;
  uptime: number;
}

export interface Waypoint {
  id: string;
  lat: number;
  lng: number;
  name: string;
  order: number;
}

export interface CostmapData {
  width: number;
  height: number;
  resolution: number;
  originX: number;
  originY: number;
  data: number[];
}

export interface VelocityCommand {
  linear: number;
  angular: number;
}

// Zustand store interface
interface RobotState {
  // Connection state
  connected: boolean;
  ros: ROSLIB.Ros | null;

  // Robot data
  stats: RobotStats;
  waypoints: Waypoint[];
  costmap: CostmapData | null;

  // Actions
  connect: (url: string) => void;
  disconnect: () => void;
  sendVelocity: (cmd: VelocityCommand) => void;
  setWaypoints: (waypoints: Waypoint[]) => void;
  addWaypoint: (waypoint: Waypoint) => void;
  removeWaypoint: (id: string) => void;
  publishWaypoints: () => void;
}

// Default stats values
const defaultStats: RobotStats = {
  temperature: 0,
  batteryLevel: 0,
  solarPower: 0,
  cpuUsage: 0,
  memoryUsage: 0,
  gpsLat: 0,
  gpsLon: 0,
  heading: 0,
  speed: 0,
  uptime: 0,
};

// Topic and service names - matches ROS2 Navigation2 stack
const TOPICS = {
  CMD_VEL: '/cmd_vel',
  ROBOT_STATS: '/robot/stats',
  GPS_FIX: '/fix',  // NMEA driver publishes to /fix
  GLOBAL_COSTMAP: '/global_costmap/costmap',
  LOCAL_COSTMAP: '/local_costmap/costmap',
  ODOM: '/odom',
  SCAN: '/scan',
  WAYPOINTS: '/navigation/waypoints',
};

// Create the Zustand store
export const useRobotStore = create<RobotState>((set, get) => {
  // Store references to subscribers for cleanup
  let statsSubscriber: ROSLIB.Topic | null = null;
  let costmapSubscriber: ROSLIB.Topic | null = null;
  let cmdVelPublisher: ROSLIB.Topic | null = null;
  let waypointPublisher: ROSLIB.Topic | null = null;

  return {
    connected: false,
    ros: null,
    stats: defaultStats,
    waypoints: [],
    costmap: null,

    connect: (url: string) => {
      const ros = new ROSLIB.Ros({ url });

      ros.on('connection', () => {
        console.log('[ROSBridge] Connected to', url);
        set({ connected: true, ros });

        // Set up subscribers
        statsSubscriber = new ROSLIB.Topic({
          ros,
          name: TOPICS.ROBOT_STATS,
          messageType: 'std_msgs/String', // JSON encoded stats
        });

        statsSubscriber.subscribe((message: ROSLIB.Message) => {
          try {
            const msg = message as { data: string };
            const stats = JSON.parse(msg.data) as RobotStats;
            set({ stats });
          } catch (error) {
            console.error('[ROSBridge] Failed to parse stats:', error);
          }
        });

        // Costmap subscriber (global costmap from Navigation2)
        costmapSubscriber = new ROSLIB.Topic({
          ros,
          name: TOPICS.GLOBAL_COSTMAP,
          messageType: 'nav_msgs/OccupancyGrid',
        });

        costmapSubscriber.subscribe((message: ROSLIB.Message) => {
          const msg = message as {
            info: {
              width: number;
              height: number;
              resolution: number;
              origin: { position: { x: number; y: number } };
            };
            data: number[];
          };
          const costmap: CostmapData = {
            width: msg.info.width,
            height: msg.info.height,
            resolution: msg.info.resolution,
            originX: msg.info.origin.position.x,
            originY: msg.info.origin.position.y,
            data: msg.data,
          };
          set({ costmap });
        });

        // Set up publishers
        cmdVelPublisher = new ROSLIB.Topic({
          ros,
          name: TOPICS.CMD_VEL,
          messageType: 'geometry_msgs/Twist',
        });

        waypointPublisher = new ROSLIB.Topic({
          ros,
          name: TOPICS.WAYPOINTS,
          messageType: 'std_msgs/String',
        });
      });

      ros.on('error', (error) => {
        console.error('[ROSBridge] Connection error:', error);
        set({ connected: false });
      });

      ros.on('close', () => {
        console.log('[ROSBridge] Connection closed');
        set({ connected: false, ros: null });
      });
    },

    disconnect: () => {
      const { ros } = get();

      // Unsubscribe from topics
      if (statsSubscriber) {
        statsSubscriber.unsubscribe();
        statsSubscriber = null;
      }
      if (costmapSubscriber) {
        costmapSubscriber.unsubscribe();
        costmapSubscriber = null;
      }

      // Close connection
      if (ros) {
        ros.close();
      }

      set({ connected: false, ros: null, stats: defaultStats, costmap: null });
    },

    sendVelocity: (cmd: VelocityCommand) => {
      if (!cmdVelPublisher) {
        console.warn('[ROSBridge] Not connected, cannot send velocity');
        return;
      }

      const twist = new ROSLIB.Message({
        linear: { x: cmd.linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: cmd.angular },
      });

      cmdVelPublisher.publish(twist);
    },

    setWaypoints: (waypoints: Waypoint[]) => {
      set({ waypoints });
    },

    addWaypoint: (waypoint: Waypoint) => {
      const { waypoints } = get();
      set({ waypoints: [...waypoints, waypoint] });
    },

    removeWaypoint: (id: string) => {
      const { waypoints } = get();
      set({ waypoints: waypoints.filter((wp) => wp.id !== id) });
    },

    publishWaypoints: () => {
      const { waypoints } = get();

      if (!waypointPublisher) {
        console.warn('[ROSBridge] Not connected, cannot publish waypoints');
        return;
      }

      const message = new ROSLIB.Message({
        data: JSON.stringify(waypoints),
      });

      waypointPublisher.publish(message);
      console.log('[ROSBridge] Published waypoints:', waypoints);
    },
  };
});

// Helper hook for velocity commands with throttling
// Fixed: Use useRef to isolate state per component instance
export const useVelocityThrottle = (intervalMs: number = 100) => {
  const lastSentRef = React.useRef(0);
  const { sendVelocity } = useRobotStore();

  return React.useCallback((cmd: VelocityCommand) => {
    const now = Date.now();
    if (now - lastSentRef.current >= intervalMs) {
      sendVelocity(cmd);
      lastSentRef.current = now;
    }
  }, [sendVelocity, intervalMs]);
};

export default useRobotStore;
