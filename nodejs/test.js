const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 8080 });

const clients = new Map(); // 클라이언트를 저장할 Map

wss.on('connection', function connection(ws, req) {
    const clientId = req.socket.remoteAddress + ":" + req.socket.remotePort;
    console.log(`New connection: ${clientId}`);

    // 클라이언트 식별자와 WebSocket 객체 연결
    clients.set(clientId, ws);

    ws.on('message', function incoming(data) {
        const parsedData = JSON.parse(data);
        console.log(`Received message from ${parsedData.deviceId}: ${parsedData.message}`);
    });

    ws.send('Hello! Message from server.');
});

wss.on('close', () => {
    clients.delete(clientId);
    console.log(`Connection closed: ${parsedData.deviceId}`);
});

console.log('WebSocket server is running on ws://192.168.0.85:8080');
