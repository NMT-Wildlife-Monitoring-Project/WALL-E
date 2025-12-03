/**
 * REST API Client Service
 * Generated/Edited by Claude
 */
import axios, { AxiosInstance, AxiosError } from 'axios';

// API response types
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
}

export interface WaypointData {
  id: string;
  lat: number;
  lng: number;
  name: string;
  order: number;
}

export interface MissionConfig {
  id: string;
  name: string;
  waypoints: WaypointData[];
  speed: number;
  returnHome: boolean;
  createdAt: string;
}

export interface SystemStatus {
  robotOnline: boolean;
  cameraOnline: boolean;
  gpsLocked: boolean;
  rosbridgeConnected: boolean;
  lastHeartbeat: string;
}

export interface LogEntry {
  timestamp: string;
  level: 'info' | 'warning' | 'error';
  message: string;
  source: string;
}

// API error class
export class ApiError extends Error {
  constructor(
    message: string,
    public statusCode?: number,
    public details?: unknown
  ) {
    super(message);
    this.name = 'ApiError';
  }
}

// Create axios instance
const createApiClient = (baseURL: string = '/api'): AxiosInstance => {
  const client = axios.create({
    baseURL,
    timeout: 10000,
    headers: {
      'Content-Type': 'application/json',
    },
  });

  // Request interceptor
  client.interceptors.request.use(
    (config) => {
      // Add auth token if available
      const token = localStorage.getItem('auth_token');
      if (token) {
        config.headers.Authorization = `Bearer ${token}`;
      }
      return config;
    },
    (error) => {
      return Promise.reject(error);
    }
  );

  // Response interceptor
  client.interceptors.response.use(
    (response) => response,
    (error: AxiosError) => {
      const apiError = new ApiError(
        error.message || 'An unexpected error occurred',
        error.response?.status,
        error.response?.data
      );
      return Promise.reject(apiError);
    }
  );

  return client;
};

const apiClient = createApiClient();

// API Service object
export const api = {
  // System endpoints
  system: {
    getStatus: async (): Promise<SystemStatus> => {
      const response = await apiClient.get<ApiResponse<SystemStatus>>('/system/status');
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to get system status');
      }
      return response.data.data;
    },

    restart: async (): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>('/system/restart');
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to restart system');
      }
    },

    shutdown: async (): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>('/system/shutdown');
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to shutdown system');
      }
    },
  },

  // Mission endpoints
  missions: {
    list: async (): Promise<MissionConfig[]> => {
      const response = await apiClient.get<ApiResponse<MissionConfig[]>>('/missions');
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to list missions');
      }
      return response.data.data;
    },

    get: async (id: string): Promise<MissionConfig> => {
      const response = await apiClient.get<ApiResponse<MissionConfig>>(`/missions/${id}`);
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to get mission');
      }
      return response.data.data;
    },

    create: async (mission: Omit<MissionConfig, 'id' | 'createdAt'>): Promise<MissionConfig> => {
      const response = await apiClient.post<ApiResponse<MissionConfig>>('/missions', mission);
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to create mission');
      }
      return response.data.data;
    },

    update: async (id: string, mission: Partial<MissionConfig>): Promise<MissionConfig> => {
      const response = await apiClient.put<ApiResponse<MissionConfig>>(`/missions/${id}`, mission);
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to update mission');
      }
      return response.data.data;
    },

    delete: async (id: string): Promise<void> => {
      const response = await apiClient.delete<ApiResponse<null>>(`/missions/${id}`);
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to delete mission');
      }
    },

    start: async (id: string): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>(`/missions/${id}/start`);
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to start mission');
      }
    },

    stop: async (): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>('/missions/stop');
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to stop mission');
      }
    },

    pause: async (): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>('/missions/pause');
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to pause mission');
      }
    },

    resume: async (): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>('/missions/resume');
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to resume mission');
      }
    },
  },

  // Waypoint endpoints
  waypoints: {
    list: async (): Promise<WaypointData[]> => {
      const response = await apiClient.get<ApiResponse<WaypointData[]>>('/waypoints');
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to list waypoints');
      }
      return response.data.data;
    },

    save: async (waypoints: WaypointData[]): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>('/waypoints', { waypoints });
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to save waypoints');
      }
    },

    clear: async (): Promise<void> => {
      const response = await apiClient.delete<ApiResponse<null>>('/waypoints');
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to clear waypoints');
      }
    },
  },

  // Camera endpoints
  camera: {
    getStreamUrl: (): string => {
      return `${apiClient.defaults.baseURL}/camera/stream`;
    },

    capture: async (): Promise<Blob> => {
      const response = await apiClient.get('/camera/capture', {
        responseType: 'blob',
      });
      return response.data;
    },

    setQuality: async (quality: 'low' | 'medium' | 'high'): Promise<void> => {
      const response = await apiClient.post<ApiResponse<null>>('/camera/quality', { quality });
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to set camera quality');
      }
    },
  },

  // Logs endpoints
  logs: {
    get: async (limit: number = 100, level?: string): Promise<LogEntry[]> => {
      const params: { limit: number; level?: string } = { limit };
      if (level) params.level = level;

      const response = await apiClient.get<ApiResponse<LogEntry[]>>('/logs', { params });
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to get logs');
      }
      return response.data.data;
    },

    clear: async (): Promise<void> => {
      const response = await apiClient.delete<ApiResponse<null>>('/logs');
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to clear logs');
      }
    },
  },

  // Config endpoints
  config: {
    get: async (): Promise<Record<string, unknown>> => {
      const response = await apiClient.get<ApiResponse<Record<string, unknown>>>('/config');
      if (!response.data.success || !response.data.data) {
        throw new ApiError(response.data.error || 'Failed to get config');
      }
      return response.data.data;
    },

    update: async (config: Record<string, unknown>): Promise<void> => {
      const response = await apiClient.put<ApiResponse<null>>('/config', config);
      if (!response.data.success) {
        throw new ApiError(response.data.error || 'Failed to update config');
      }
    },
  },
};

export default api;
