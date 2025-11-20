const mqtt = require('mqtt');
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');

// MQTT Configuration - update this to match your broker
const MQTT_BROKER = 'mqtt://localhost:1883';
const client = mqtt.connect(MQTT_BROKER);

// Express setup
const app = express();
const server = http.createServer(app);
const io = socketIo(server);

app.use(express.static('public'));

// MQTT connection
client.on('connect', () => {
    console.log('Connected to MQTT broker');
    client.subscribe('robot/line/#');
    client.subscribe('robot/barcode');
    client.subscribe('robot/obstacle');
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

server.listen(3000, () => {
    console.log('Dashboard running on http://localhost:3000');
});