/**
 * CameraFeed Component - Displays MJPEG stream from backend
 * Generated/Edited by Claude
 */
import React, { useState, useEffect, useCallback, useRef } from 'react';

interface CameraFeedProps {
  streamUrl: string;
  refreshInterval?: number;
  showControls?: boolean;
}

type StreamStatus = 'loading' | 'connected' | 'error' | 'paused';

const CameraFeed: React.FC<CameraFeedProps> = ({
  streamUrl,
  refreshInterval = 0,
  showControls = true,
}) => {
  const [status, setStatus] = useState<StreamStatus>('loading');
  const [errorMessage, setErrorMessage] = useState<string>('');
  const [isPaused, setIsPaused] = useState(false);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const imgRef = useRef<HTMLImageElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const retryCountRef = useRef(0);
  const isMountedRef = useRef(true);
  const maxRetries = 5;

  // Track mounted state to prevent state updates after unmount
  useEffect(() => {
    isMountedRef.current = true;
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // Generate unique URL to force refresh (for non-MJPEG fallback)
  const getStreamUrlWithTimestamp = useCallback(() => {
    if (refreshInterval > 0) {
      return `${streamUrl}?t=${Date.now()}`;
    }
    return streamUrl;
  }, [streamUrl, refreshInterval]);

  // Handle image load success
  const handleLoad = useCallback(() => {
    setStatus('connected');
    setErrorMessage('');
    retryCountRef.current = 0;
  }, []);

  // Handle image load error
  // Fixed: Check mounted state before state updates to prevent memory leaks
  const handleError = useCallback(() => {
    if (!isMountedRef.current) return;

    retryCountRef.current += 1;

    if (retryCountRef.current >= maxRetries) {
      setStatus('error');
      setErrorMessage(`Failed to connect to camera stream after ${maxRetries} attempts`);
    } else {
      setErrorMessage(`Connection attempt ${retryCountRef.current}/${maxRetries}...`);
      // Retry after a delay with mounted check
      setTimeout(() => {
        if (isMountedRef.current && imgRef.current && !isPaused) {
          imgRef.current.src = getStreamUrlWithTimestamp();
        }
      }, 2000);
    }
  }, [isPaused, getStreamUrlWithTimestamp]);

  // Retry connection
  const handleRetry = useCallback(() => {
    retryCountRef.current = 0;
    setStatus('loading');
    setErrorMessage('');
    if (imgRef.current) {
      imgRef.current.src = getStreamUrlWithTimestamp();
    }
  }, [getStreamUrlWithTimestamp]);

  // Toggle pause
  const togglePause = useCallback(() => {
    setIsPaused((prev) => {
      if (prev) {
        // Resume
        setStatus('loading');
        if (imgRef.current) {
          imgRef.current.src = getStreamUrlWithTimestamp();
        }
      } else {
        // Pause
        setStatus('paused');
      }
      return !prev;
    });
  }, [getStreamUrlWithTimestamp]);

  // Toggle fullscreen
  const toggleFullscreen = useCallback(() => {
    if (!containerRef.current) return;

    if (!isFullscreen) {
      if (containerRef.current.requestFullscreen) {
        containerRef.current.requestFullscreen();
      }
    } else {
      if (document.exitFullscreen) {
        document.exitFullscreen();
      }
    }
  }, [isFullscreen]);

  // Listen for fullscreen changes
  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(!!document.fullscreenElement);
    };

    document.addEventListener('fullscreenchange', handleFullscreenChange);
    return () => {
      document.removeEventListener('fullscreenchange', handleFullscreenChange);
    };
  }, []);

  // Set up refresh interval for JPEG mode
  useEffect(() => {
    if (refreshInterval > 0 && !isPaused) {
      const interval = setInterval(() => {
        if (imgRef.current) {
          imgRef.current.src = getStreamUrlWithTimestamp();
        }
      }, refreshInterval);

      return () => clearInterval(interval);
    }
  }, [refreshInterval, isPaused, getStreamUrlWithTimestamp]);

  // Render loading state
  if (status === 'loading') {
    return (
      <div className="camera-container" ref={containerRef}>
        <div className="loading-container">
          <div className="loading-spinner"></div>
          <p>Connecting to camera...</p>
          {errorMessage && <p style={{ fontSize: '0.875rem' }}>{errorMessage}</p>}
        </div>
        <img
          ref={imgRef}
          src={getStreamUrlWithTimestamp()}
          alt="Camera Feed"
          onLoad={handleLoad}
          onError={handleError}
          style={{ display: 'none' }}
        />
      </div>
    );
  }

  // Render error state
  if (status === 'error') {
    return (
      <div className="camera-container" ref={containerRef}>
        <div className="error-container">
          <svg
            width="48"
            height="48"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
          >
            <circle cx="12" cy="12" r="10" />
            <line x1="12" y1="8" x2="12" y2="12" />
            <line x1="12" y1="16" x2="12.01" y2="16" />
          </svg>
          <p>{errorMessage}</p>
          <button onClick={handleRetry}>Retry Connection</button>
        </div>
      </div>
    );
  }

  // Render connected/paused state
  return (
    <div className="camera-container" ref={containerRef}>
      <div className="camera-feed-wrapper">
        {status === 'paused' ? (
          <div className="paused-overlay">
            <svg
              width="64"
              height="64"
              viewBox="0 0 24 24"
              fill="currentColor"
            >
              <rect x="6" y="4" width="4" height="16" />
              <rect x="14" y="4" width="4" height="16" />
            </svg>
            <p>Stream Paused</p>
          </div>
        ) : (
          <img
            ref={imgRef}
            src={getStreamUrlWithTimestamp()}
            alt="Camera Feed"
            onLoad={handleLoad}
            onError={handleError}
          />
        )}

        {showControls && (
          <div className="camera-controls">
            <button
              className="camera-control-btn"
              onClick={togglePause}
              title={isPaused ? 'Resume' : 'Pause'}
            >
              {isPaused ? (
                <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                  <polygon points="5,3 19,12 5,21" />
                </svg>
              ) : (
                <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                  <rect x="6" y="4" width="4" height="16" />
                  <rect x="14" y="4" width="4" height="16" />
                </svg>
              )}
            </button>
            <button
              className="camera-control-btn"
              onClick={toggleFullscreen}
              title={isFullscreen ? 'Exit Fullscreen' : 'Fullscreen'}
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                {isFullscreen ? (
                  <>
                    <polyline points="4,14 4,20 10,20" />
                    <polyline points="20,10 20,4 14,4" />
                    <line x1="14" y1="10" x2="20" y2="4" />
                    <line x1="4" y1="20" x2="10" y2="14" />
                  </>
                ) : (
                  <>
                    <polyline points="15,3 21,3 21,9" />
                    <polyline points="9,21 3,21 3,15" />
                    <line x1="21" y1="3" x2="14" y2="10" />
                    <line x1="3" y1="21" x2="10" y2="14" />
                  </>
                )}
              </svg>
            </button>
            <button
              className="camera-control-btn"
              onClick={handleRetry}
              title="Refresh"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polyline points="23,4 23,10 17,10" />
                <path d="M20.49,15a9,9,0,1,1-2.12-9.36L23,10" />
              </svg>
            </button>
          </div>
        )}
      </div>

      <style>{`
        .camera-feed-wrapper {
          position: relative;
          width: 100%;
          height: 100%;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .camera-feed-wrapper img {
          max-width: 100%;
          max-height: 100%;
          object-fit: contain;
        }

        .paused-overlay {
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
          color: var(--text-secondary);
        }

        .paused-overlay p {
          margin-top: var(--spacing-md);
        }

        .camera-controls {
          position: absolute;
          bottom: var(--spacing-md);
          right: var(--spacing-md);
          display: flex;
          gap: var(--spacing-sm);
        }

        .camera-control-btn {
          width: 36px;
          height: 36px;
          display: flex;
          align-items: center;
          justify-content: center;
          background-color: rgba(22, 27, 34, 0.8);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          color: var(--text-primary);
          cursor: pointer;
          transition: background-color 0.2s;
        }

        .camera-control-btn:hover {
          background-color: rgba(33, 38, 45, 0.9);
        }
      `}</style>
    </div>
  );
};

export default CameraFeed;
