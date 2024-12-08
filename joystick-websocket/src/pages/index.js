import { useState, useRef } from 'react';
import dynamic from 'next/dynamic';

// Dynamically import Joystick component to avoid SSR issues
const Joystick = dynamic(
  () => import('react-joystick-component').then((mod) => mod.Joystick),
  { ssr: false }
);

export default function Home() {
  const [ws, setWs] = useState(null); // WebSocket instance
  const [isConnected, setIsConnected] = useState(false); // Connection status
  const [armed, setArmed] = useState(false); // Arm status
  const lastSentCommand = useRef({ linear: { x: 0.0 }, angular: { z: 0.0 } }); // Keep track of the last sent command
  const intervalId = useRef(null); // For throttling joystick commands

  // Handle WebSocket connection
  const connectWebSocket = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      console.log('WebSocket is already connected');
      return;
    }

    const socket = new WebSocket('ws://localhost:8080');
    setWs(socket);

    socket.onopen = () => {
      console.log('WebSocket connected');
      setIsConnected(true);
    };

    socket.onclose = () => {
      console.log('WebSocket disconnected');
      setIsConnected(false);
      clearInterval(intervalId.current);
    };

    socket.onerror = (error) => {
      console.error('WebSocket error:', error);
    };
  };

  // Handle WebSocket disconnection
  const disconnectWebSocket = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.close();
    } else {
      console.log('WebSocket is not connected');
    }
  };

  // Throttle joystick command sending
  const sendThrottledCommand = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(lastSentCommand.current));
      console.log('Sent command:', lastSentCommand.current);
    } else {
      console.warn('WebSocket is not connected');
    }
  };

  // Handle joystick move event
  const handleMove = (data) => {
    // Normalize values and update the last sent command
    lastSentCommand.current = {
      linear: { x: parseFloat((data.y).toFixed(2)) },
      angular: { z: parseFloat((-data.x).toFixed(2)) },
    };
  };

  // Handle joystick stop event
  const handleStop = () => {
    // Reset the last sent command to zero values
    lastSentCommand.current = { linear: { x: 0.0 }, angular: { z: 0.0 } };
  };

  // Handle arm/disarm toggle
  const toggleArm = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      const armCommand = { arm: !armed }; // Toggle arm state
      ws.send(JSON.stringify(armCommand));
      console.log(`Sent: ${armed ? 'Disarm' : 'Arm'}`);
      setArmed(!armed); // Update local arm state
    } else {
      console.warn('WebSocket is not connected');
    }
  };

  // Start throttling commands on joystick use
  const startThrottling = () => {
    if (!intervalId.current) {
      intervalId.current = setInterval(sendThrottledCommand, 200); // Send commands every 0.2s
    }
  };

  // Stop throttling commands when joystick stops
  const stopThrottling = () => {
    clearInterval(intervalId.current);
    intervalId.current = null;
    handleStop();
    sendThrottledCommand(); // Send a final stop command immediately
  };

  return (
    <div style={{ textAlign: 'center', marginTop: '50px' }}>
      <h1>Joystick Control</h1>

      <div style={{ marginBottom: '20px' }}>
        <button
          onClick={toggleArm}
          disabled={!isConnected}
          style={{
            padding: '10px 20px',
            marginRight: '10px',
            cursor: !isConnected ? 'not-allowed' : 'pointer',
            backgroundColor: armed ? '#f44336' : '#4CAF50',
            color: 'white',
            border: 'none',
            borderRadius: '5px',
          }}
        >
          {armed ? 'Disarm' : 'Arm'}
        </button>

        <button
          onClick={connectWebSocket}
          disabled={isConnected}
          style={{
            padding: '10px 20px',
            marginRight: '10px',
            cursor: isConnected ? 'not-allowed' : 'pointer',
            backgroundColor: isConnected ? '#ccc' : '#4CAF50',
            color: 'white',
            border: 'none',
            borderRadius: '5px',
          }}
        >
          Connect
        </button>

        <button
          onClick={disconnectWebSocket}
          disabled={!isConnected}
          style={{
            padding: '10px 20px',
            cursor: !isConnected ? 'not-allowed' : 'pointer',
            backgroundColor: !isConnected ? '#ccc' : '#f44336',
            color: 'white',
            border: 'none',
            borderRadius: '5px',
          }}
        >
          Disconnect
        </button>
      </div>

      {/* Joystick container */}
      <div
        style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '300px',
        }}
      >
        <Joystick
          size={150}
          baseColor="lightgray"
          stickColor="black"
          move={(data) => {
            handleMove(data);
            startThrottling(); // Start throttling commands when the joystick moves
          }}
          stop={() => {
            stopThrottling(); // Stop throttling commands when the joystick stops
          }}
        />
      </div>

      <p>Status: {isConnected ? 'Connected' : 'Disconnected'}</p>
    </div>
  );
}
