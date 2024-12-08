const WebSocket = require('ws');

// Start the WebSocket server
const wss = new WebSocket.Server({ port: 8080 });

wss.on('connection', (ws) => {
  console.log('WebSocket client connected');

  ws.on('message', (message) => {
    console.log('Received:', message);
    // Here you can forward cmd_vel messages to the robot
  });

  ws.on('close', () => {
    console.log('WebSocket client disconnected');
  });
});

console.log('WebSocket server running on ws://localhost:8080');
