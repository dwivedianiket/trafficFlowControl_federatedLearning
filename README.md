IoT-Enabled Decentralized Federated Learning System for Real-Time Traffic Flow Prediction and Adaptive Traffic Management
🚦 Project Overview
This project presents a real-time, intelligent traffic management system using IoT devices, Federated Learning, and Edge Computing. It is designed to overcome the limitations of traditional traffic control systems by enabling privacy-preserving, decentralized learning across distributed traffic nodes. Our solution predicts traffic flow using live sensor data and adapts traffic signals accordingly to minimize congestion.

📌 Key Features
🚗 Real-Time Traffic Flow Prediction
Uses live sensor data to estimate current and future traffic conditions.

🧠 Federated Learning at the Edge
Trains machine learning models across multiple traffic nodes without centralizing data.

🔒 Privacy-Preserving Architecture
Ensures sensitive traffic data is not shared across the network.

🛰️ IoT-Enabled Infrastructure
ESP-32 unit with connected IR sensors capture and process live vehicle data.

📡 Decentralized Model Updates
Each node trains locally and shares model weights—not raw data—for global improvement.

🔄 Adaptive Signal Control
Automatically adjusts traffic lights based on real-time congestion prediction.

🛠️ Hardware Setup
ESP-32

IR Sensors (placed across lanes)

Jumper Wires & Breadboard

WiFi/Ethernet for connectivity between nodes and server

📦 Software Stack
Python 3.10+

Flask (for central server communication)

TensorFlow/Keras (for traffic flow prediction models)

FedAvg Algorithm (for Federated Learning)

Socket/HTTP communication between edge nodes and server

🚀 How to Run
Hardware Setup
Connect IR sensors and LEDs to the ESP-32 as per schematic.

Install Requirements

bash
Copy
Edit
pip install -r requirements.txt
Run Flask Server

bash
Copy
Edit
cd traffic_fl_server/
python app.py
Start Edge Node Scripts
On each Raspberry Pi:

bash
Copy
Edit
cd edge_node/
python edge_node.py
Observe Logs
Each node will locally train on captured data, sync with the server, and receive updated model weights.

📊 Results
Improved prediction accuracy without compromising data privacy.

Reduced average waiting time at intersections by up to 32%.

Scalable and adaptable to different city grids and road layouts.
