#!/usr/bin/env python3
"""
Test MQTT publisher - simulates the robot car publishing telemetry data
Run this to test the dashboard without the physical robot
"""

import paho.mqtt.client as mqtt
import json
import time
import random
import math

# MQTT Configuration
MQTT_BROKER = "localhost"  # Change to your broker IP or localhost
MQTT_PORT = 1883

# Topics (matching the robot)
TOPIC_STATE = "robot/line/state"
TOPIC_ERROR = "robot/line/error"
TOPIC_SPEED = "robot/line/speed"
TOPIC_POSITION = "robot/line/position"
TOPIC_BARCODE = "robot/barcode"
TOPIC_OBSTACLE = "robot/obstacle"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker successfully!")
    else:
        print(f"Connection failed with code {rc}")

def simulate_robot():
    client = mqtt.Client(client_id="robot_simulator")
    client.on_connect = on_connect
    
    try:
        print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
        
        time.sleep(1)  # Wait for connection
        
        print("Starting robot simulation... (Press Ctrl+C to stop)")
        
        distance = 0.0
        t = 0
        barcode_counter = 0
        
        while True:
            # Simulate state (alternating between STRAIGHT and TURNING)
            state = "TURNING" if (t % 20) > 15 else "STRAIGHT"
            edge = "LEFT" if state == "STRAIGHT" else "RIGHT"
            heading = (t * 5) % 360
            
            state_data = {
                "state": state,
                "edge": edge,
                "heading": heading
            }
            client.publish(TOPIC_STATE, json.dumps(state_data))
            
            # Simulate error with sine wave
            error = math.sin(t * 0.1) * 0.5
            correction = error * 0.3
            p = error * 0.20
            i = random.uniform(-0.05, 0.05)
            d = random.uniform(-0.02, 0.02)
            
            error_data = {
                "error": round(error, 3),
                "correction": round(correction, 3),
                "p": round(p, 2),
                "i": round(i, 2),
                "d": round(d, 2)
            }
            client.publish(TOPIC_ERROR, json.dumps(error_data))
            
            # Simulate motor speeds
            base = 0.25
            left = max(0, min(1, base - correction))
            right = max(0, min(1, base + correction))
            
            speed_data = {
                "left": round(left, 2),
                "right": round(right, 2),
                "base": base
            }
            client.publish(TOPIC_SPEED, json.dumps(speed_data))
            
            # Simulate position
            distance += 0.01
            position_data = {
                "distance": round(distance, 2)
            }
            client.publish(TOPIC_POSITION, json.dumps(position_data))
            
            # Occasionally publish a barcode
            if t % 30 == 0 and t > 0:
                barcodes = ["A123", "B456", "C789", "D012", "E345"]
                barcode = random.choice(barcodes)
                barcode_data = {
                    "barcode": barcode
                }
                client.publish(TOPIC_BARCODE, json.dumps(barcode_data))
                print(f"  📊 Published barcode: {barcode}")
            
            # Simulate obstacle detection (randomly every 15-25 seconds)
            if t % 40 == 0:
                # Obstacle detected
                obstacle_data = {
                    "detected": True,
                    "distance": random.uniform(5.0, 50.0),
                    "width": random.uniform(3.0, 20.0)
                }
                client.publish(TOPIC_OBSTACLE, json.dumps(obstacle_data))
                print(f"  🚧 Obstacle detected: {obstacle_data['distance']:.1f}cm away, {obstacle_data['width']:.1f}cm wide")
            elif t % 40 == 5:
                # Obstacle cleared
                obstacle_data = {
                    "detected": False,
                    "distance": 0,
                    "width": 0
                }
                client.publish(TOPIC_OBSTACLE, json.dumps(obstacle_data))
                print(f"  ✅ Obstacle cleared")
            
            print(f"  ✓ Published telemetry (t={t}, state={state}, distance={distance:.2f}m)")
            
            t += 1
            time.sleep(0.2)  # Publish every 200ms (5Hz)
            
    except KeyboardInterrupt:
        print("\n\nStopping simulation...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("Disconnected from MQTT broker")

if __name__ == "__main__":
    print("=" * 60)
    print("MQTT Robot Simulator")
    print("=" * 60)
    print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print("Topics:")
    print(f"  - {TOPIC_STATE}")
    print(f"  - {TOPIC_ERROR}")
    print(f"  - {TOPIC_SPEED}")
    print(f"  - {TOPIC_POSITION}")
    print(f"  - {TOPIC_BARCODE}")
    print(f"  - {TOPIC_OBSTACLE}")
    print("=" * 60)
    print()
    
    simulate_robot()
