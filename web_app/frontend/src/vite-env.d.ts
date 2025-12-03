/**
 * Vite Environment Type Declarations
 * Generated/Edited by Claude
 */

/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_GOOGLE_MAPS_API_KEY: string;
  readonly VITE_ROSBRIDGE_URL: string;
  readonly VITE_API_BASE_URL: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}

// Type augmentations for libraries without complete types
declare module 'roslib' {
  export class Ros {
    constructor(options: { url: string });
    on(event: 'connection' | 'error' | 'close', callback: (error?: Error) => void): void;
    close(): void;
  }

  export class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
    });
    subscribe(callback: (message: Message) => void): void;
    unsubscribe(): void;
    publish(message: Message): void;
  }

  export class Message {
    constructor(values: Record<string, unknown>);
  }

  export class Service {
    constructor(options: {
      ros: Ros;
      name: string;
      serviceType: string;
    });
    callService(request: ServiceRequest, callback: (response: unknown) => void): void;
  }

  export class ServiceRequest {
    constructor(values: Record<string, unknown>);
  }

  export class Param {
    constructor(options: {
      ros: Ros;
      name: string;
    });
    get(callback: (value: unknown) => void): void;
    set(value: unknown, callback?: () => void): void;
  }
}

declare module 'nipplejs' {
  export interface JoystickOutputData {
    angle: {
      degree: number;
      radian: number;
    };
    direction: {
      angle: string;
      x: string;
      y: string;
    };
    distance: number;
    force: number;
    identifier: number;
    instance: unknown;
    position: {
      x: number;
      y: number;
    };
    pressure: number;
    vector: {
      x: number;
      y: number;
    };
  }

  export interface JoystickManagerOptions {
    zone: HTMLElement;
    mode?: 'static' | 'semi' | 'dynamic';
    position?: { left?: string; top?: string; right?: string; bottom?: string };
    color?: string;
    size?: number;
    threshold?: number;
    fadeTime?: number;
    multitouch?: boolean;
    maxNumberOfNipples?: number;
    dataOnly?: boolean;
    restOpacity?: number;
    restJoystick?: boolean | object;
    catchDistance?: number;
    lockX?: boolean;
    lockY?: boolean;
  }

  export interface JoystickManager {
    on(
      event: 'start' | 'end' | 'move' | 'dir' | 'plain' | 'shown' | 'hidden' | 'destroyed',
      handler: (evt: unknown, data: JoystickOutputData) => void
    ): void;
    off(event: string, handler?: unknown): void;
    destroy(): void;
    get(id?: number): unknown;
    ids: number[];
  }

  export function create(options: JoystickManagerOptions): JoystickManager;
}
