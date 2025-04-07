import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from datetime import datetime
import logging
from flask import request, redirect, url_for
from flask import Flask, jsonify, render_template, send_from_directory
import os

# Initialize Flask
app = Flask(__name__)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Initialize Firebase
cred = credentials.Certificate(r"C:\Users\LENOVO\PycharmProjects\pythonProject1\traffic_fl_server\firebase-credentials.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://traffic-new-90b5d-default-rtdb.firebaseio.com/'
})

# Track previous vehicle counts
previous_counts = {}


def get_latest_traffic_data():
    """Fetch the most recent traffic data for each junction from Firebase"""
    try:
        # Get reference to traffic_data
        traffic_ref = db.reference('traffic_data')
        firebase_data = traffic_ref.get() or {}

        result = {}

        # Process each junction
        for junction_id, timestamps in firebase_data.items():
            if not timestamps:
                continue

            # Find the most recent timestamp
            latest_timestamp = max(timestamps.keys())

            # Get data for that timestamp
            junction_data = timestamps[latest_timestamp]

            # Store this as the current data for this junction
            result[junction_id] = junction_data

            # Add the timestamp from the key
            result[junction_id]['timestamp'] = latest_timestamp

            # Initialize previous counts if not exists
            if junction_id not in previous_counts:
                previous_counts[junction_id] = {
                    'vehicle_count1': junction_data.get('vehicle_count1', 0),
                    'vehicle_count2': junction_data.get('vehicle_count2', 0)
                }

        return result
    except Exception as e:
        logging.error(f"Error fetching traffic data from Firebase: {str(e)}")
        return {}


@app.route('/')
def index():
    """Serve the main dashboard page"""
    return render_template('index.html')


@app.route('/api/light-states')
def get_light_states():
    """Provide current light states for hardware controllers"""
    try:
        # Get traffic data
        traffic_data = get_latest_traffic_data()

        if not traffic_data:
            return jsonify({"error": "No data available"}), 404

        # Get the first junction (or specific junction if needed)
        junction_id = next(iter(traffic_data))
        junction_data = traffic_data[junction_id]

        # Determine signal states (same logic as your dashboard)
        vehicle_count1 = int(junction_data.get('vehicle_count1', 0))
        vehicle_count2 = int(junction_data.get('vehicle_count2', 0))

        # Calculate differences from previous counts
        diff1 = vehicle_count1 - previous_counts[junction_id]['vehicle_count1']
        diff2 = vehicle_count2 - previous_counts[junction_id]['vehicle_count2']

        # Determine signal state (same logic as your web interface)
        signal_color = "red"
        if abs(diff1) > 5 or abs(diff2) > 5:
            signal_color = "green" if diff1 > diff2 else "red"

        return jsonify({
            "horizontal": signal_color if diff1 > diff2 else "red",
            "vertical": "green" if diff1 > diff2 else signal_color,
            "last_updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
    except Exception as e:
        logging.error(f"Error in light-states: {str(e)}")
        return jsonify({"error": str(e)}), 500



@app.route('/dashboard_data', methods=['GET'])
def dashboard_data():
    """Provide data for the dashboard from Firebase"""
    try:
        # Get traffic data
        traffic_data = get_latest_traffic_data()

        # Prepare response data
        response_data = {
            "global_model": {
                "version": "1.0",
                "last_updated": datetime.now().strftime("%Y%m%d%H%M%S")
            },
            "junction_count": len(traffic_data),
            "junctions": {},
            "last_update": datetime.now().strftime("%Y%m%d%H%M%S"),
            "total_vehicles": 0
        }

        # Process each junction's data
        for junction_id, data in traffic_data.items():
            # Get current counts
            vehicle_count1 = int(data.get('vehicle_count1', 0))
            vehicle_count2 = int(data.get('vehicle_count2', 0))

            # Calculate differences from previous counts
            diff1 = vehicle_count1 - previous_counts[junction_id]['vehicle_count1']
            diff2 = vehicle_count2 - previous_counts[junction_id]['vehicle_count2']

            # Determine signal state based on vehicle counts (difference > 5)
            signal_color = "red"
            if abs(diff1) > 5 or abs(diff2) > 5:
                signal_color = "green" if diff1 > diff2 else "red"

            # Update response data
            response_data['junctions'][junction_id] = {
                "vehicle_count1": vehicle_count1,
                "vehicle_count2": vehicle_count2,
                "ultrasonic1": int(data.get('ultrasonic1', 0)),
                "ultrasonic2": int(data.get('ultrasonic2', 0)),
                "signal_state": {
                    "color": signal_color,
                    "duration": 30  # Fixed duration for simplicity
                },
                "timestamp": data.get('timestamp', ""),
                "difference1": diff1,
                "difference2": diff2
            }

            # Update total vehicles
            response_data['total_vehicles'] += vehicle_count1 + vehicle_count2

            # Update previous counts
            previous_counts[junction_id] = {
                'vehicle_count1': vehicle_count1,
                'vehicle_count2': vehicle_count2
            }

        return jsonify(response_data)
    except Exception as e:
        logging.error(f"Error in dashboard_data: {str(e)}")
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)