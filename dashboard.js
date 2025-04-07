// Simple Traffic Light Dashboard
document.addEventListener('DOMContentLoaded', function() {
    // First make sure we can see the test signal
    console.log("Dashboard loaded - you should see a TEST traffic light");

    // Then load real data
    loadRealData();
});

function loadRealData() {
    fetch('/dashboard_data')
        .then(response => response.json())
        .then(data => {
            console.log("Data received from server:", data);
            updateTrafficSignals(data);
        })
        .catch(error => {
            console.error("Error loading data:", error);
            // Keep showing the test signal if real data fails
        });
}

function updateTrafficSignals(data) {
    const container = document.getElementById('traffic-signals-container');

    // Clear the test signal if we have real data
    if (data.junctions && Object.keys(data.junctions).length > 0) {
        container.innerHTML = '';
    }

    // Add real signals from data
    if (data.junctions) {
        Object.entries(data.junctions).forEach(([junctionId, junctionData]) => {
            const signalState = junctionData.signal_state || { color: 'red', duration: 30 };

            const signalDiv = document.createElement('div');
            signalDiv.className = 'traffic-signal';
            signalDiv.innerHTML = `
                <div class="signal-junction-label">Junction: ${junctionId}</div>
                <div class="signal-lights">
                    <div class="signal-light red ${signalState.color === 'red' ? 'active' : ''}"></div>
                    <div class="signal-light yellow ${signalState.color === 'yellow' ? 'active' : ''}"></div>
                    <div class="signal-light green ${signalState.color === 'green' ? 'active' : ''}"></div>
                </div>
                <div class="signal-timer">${signalState.duration}s</div>
                <div class="sensor-data">
                    <p>IR Sensor 1: <span>${junctionData.ir1 || 0}</span> vehicles</p>
                    <p>IR Sensor 2: <span>${junctionData.ir2 || 0}</span> vehicles</p>
                </div>
            `;

            container.appendChild(signalDiv);
        });
    }
}