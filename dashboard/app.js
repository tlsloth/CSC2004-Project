const mqtt = require('mqtt');
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');

// MQTT Configuration - update this to match your broker
const MQTT_BROKER = 'mqtt://172.20.10.2:1883';
const client = mqtt.connect(MQTT_BROKER);

let mqttConnected = false;

// Express setup
const app = express();
const server = http.createServer(app);
const io = socketIo(server);

app.use(express.static('public'));

// MQTT connection
client.on('connect', () => {
    mqttConnected = true;
    console.log('Connected to MQTT broker');
    io.emit('mqtt-status', { connected: true });
    client.subscribe('robot/line/#');
    client.subscribe('robot/barcode');
    client.subscribe('robot/obstacle');
});

client.on('error', (err) => {
    mqttConnected = false;
    console.error('MQTT connection error:', err.message);
    io.emit('mqtt-status', { connected: false });
});

client.on('close', () => {
    mqttConnected = false;
    console.log('MQTT connection closed');
    io.emit('mqtt-status', { connected: false });
});

// Forward MQTT messages to web clients
client.on('message', (topic, message) => {
    try {
        const data = JSON.parse(message.toString());
        io.emit(topic, data);
    } catch (e) {
        console.error('Parse error:', e);
    }
});

// Send MQTT status to new clients
io.on('connection', (socket) => {
    socket.emit('mqtt-status', { connected: mqttConnected });
});

server.listen(3000, () => {
    console.log('Dashboard running on http://localhost:3000');
});