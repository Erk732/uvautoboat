// ============================================================================
// Robust Avoidance Dashboard - Main JavaScript
// ============================================================================

// ROS Connection
let ros;
let connected = false;

// Subscribers
let poseSubscriber;
let goalSubscriber;
let thrustLeftSubscriber;
let thrustRightSubscriber;
let scanSubscriber;

// Publishers
let goalPublisher;

// Camera (no need for cameraUpdateInterval - MJPEG is continuous)

// Map
let map;
let boatMarker;
let goalMarker;
let trajectoryLine;
let trajectoryPoints = [];
let mapFollowBoat = true;

// Node logs subscriber
let nodeLogsSubscriber;

// ============================================================================
// Initialize Map
// ============================================================================
function initMap() {
    // Initialize map at Sydney Regatta Centre approximate location
    map = L.map('map').setView([-33.8361, 151.0697], 16);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);

    // Custom boat icon
    const boatIcon = L.divIcon({
        className: 'boat-marker',
        html: '🚤',
        iconSize: [30, 30],
        iconAnchor: [15, 15]
    });

    boatMarker = L.marker([-33.8361, 151.0697], { icon: boatIcon }).addTo(map);
    trajectoryLine = L.polyline([], { color: '#667eea', weight: 3 }).addTo(map);

    // Add follow boat toggle button
    const FollowBoatControl = L.Control.extend({
        options: { position: 'topright' },
        onAdd: function(map) {
            const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control follow-boat-control');
            const button = L.DomUtil.create('a', 'follow-boat-btn', container);
            button.href = '#';
            button.title = 'Follow boat';
            button.innerHTML = '🎯';
            button.id = 'follow-boat-toggle';

            L.DomEvent.on(button, 'click', function(e) {
                L.DomEvent.preventDefault(e);
                L.DomEvent.stopPropagation(e);
                mapFollowBoat = !mapFollowBoat;
                button.className = mapFollowBoat ? 'follow-boat-btn active' : 'follow-boat-btn';
                if (mapFollowBoat && boatMarker) {
                    map.panTo(boatMarker.getLatLng());
                }
            });

            // Set initial state
            button.className = 'follow-boat-btn active';

            return container;
        }
    });

    map.addControl(new FollowBoatControl());
    log('Map initialized', 'info');
}

// ============================================================================
// Initialize ROS Connection
// ============================================================================
function initROS() {
    ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    ros.on('connection', function() {
        console.log('Connected to ROS bridge');
        connected = true;
        updateConnectionStatus(true);
        log('Connected to ROS bridge', 'success');

        // Initialize subscribers and publishers
        initSubscribers();
        initPublishers();
        initCamera();
    });

    ros.on('error', function(error) {
        console.error('Error connecting to ROS bridge:', error);
        log('Error connecting to ROS bridge: ' + error, 'error');
        updateConnectionStatus(false);
    });

    ros.on('close', function() {
        console.log('Connection to ROS bridge closed');
        connected = false;
        updateConnectionStatus(false);
        log('Connection to ROS bridge closed', 'warning');

        // Try to reconnect after 3 seconds
        setTimeout(initROS, 3000);
    });
}

// ============================================================================
// Initialize Subscribers
// ============================================================================
function initSubscribers() {
    // Subscribe to filtered pose
    poseSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/pose_filtered',
        messageType: 'geometry_msgs/PoseStamped'
    });

    poseSubscriber.subscribe(function(message) {
        updatePoseDisplay(message);
    });

    // Subscribe to left thruster
    thrustLeftSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/left/thrust',
        messageType: 'std_msgs/Float32'
    });

    thrustLeftSubscriber.subscribe(function(message) {
        updateThrustDisplay('left', message.data);
    });

    // Subscribe to right thruster
    thrustRightSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/right/thrust',
        messageType: 'std_msgs/Float32'
    });

    thrustRightSubscriber.subscribe(function(message) {
        updateThrustDisplay('right', message.data);
    });

    // Subscribe to LiDAR scan
    scanSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
        messageType: 'sensor_msgs/LaserScan'
    });

    scanSubscriber.subscribe(function(message) {
        processLidarScan(message);
    });

    // Subscribe to planning goal
    goalSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/planning/goal',
        messageType: 'geometry_msgs/PoseStamped'
    });

    goalSubscriber.subscribe(function(message) {
        log('Goal received: (' + message.pose.position.x.toFixed(1) + ', ' +
            message.pose.position.y.toFixed(1) + ')', 'info');
    });

    // Subscribe to GPS fix for map updates
    const gpsSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/sensors/gps/gps/fix',
        messageType: 'sensor_msgs/NavSatFix'
    });

    gpsSubscriber.subscribe(function(message) {
        updateMapPosition(message.latitude, message.longitude);
    });

    // Subscribe to ROS logs for node feedback
    nodeLogsSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/rosout',
        messageType: 'rcl_interfaces/Log'
    });

    nodeLogsSubscriber.subscribe(function(message) {
        // Only show logs from robust_avoidance node
        if (message.name === '/robust_avoidance') {
            addNodeLog(message);
        }
    });
}

// ============================================================================
// Initialize Publishers
// ============================================================================
function initPublishers() {
    goalPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/planning/goal',
        messageType: 'geometry_msgs/PoseStamped'
    });
}

// ============================================================================
// Update Display Functions
// ============================================================================
function updatePoseDisplay(message) {
    const x = message.pose.position.x;
    const y = message.pose.position.y;

    // Calculate heading from quaternion
    const qz = message.pose.orientation.z;
    const qw = message.pose.orientation.w;
    const heading = Math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz)) * 180.0 / Math.PI;

    document.getElementById('local-x').textContent = x.toFixed(2) + ' m';
    document.getElementById('local-y').textContent = y.toFixed(2) + ' m';
    document.getElementById('heading').textContent = heading.toFixed(1) + '°';

    // Update GPS status
    const gpsStatus = document.getElementById('gps-status');
    gpsStatus.textContent = 'Active';
    gpsStatus.className = 'value badge success';
}

function updateThrustDisplay(side, value) {
    const thrustElement = document.getElementById(side + '-thrust');
    const barElement = document.getElementById(side + '-thrust-bar');

    thrustElement.textContent = value.toFixed(1) + ' N';

    // Update thrust bar (scale -1000 to 1000 N)
    const percentage = Math.abs(value) / 1000 * 100;
    const clampedPercentage = Math.min(percentage, 100);
    barElement.style.width = clampedPercentage + '%';

    if (value > 0) {
        barElement.style.backgroundColor = '#2ecc71'; // Green for forward
    } else if (value < 0) {
        barElement.style.backgroundColor = '#e74c3c'; // Red for reverse
    } else {
        barElement.style.backgroundColor = '#95a5a6'; // Gray for zero
    }
}

function processLidarScan(message) {
    const ranges = message.ranges;
    const angleMin = message.angle_min;
    const angleIncrement = message.angle_increment;

    let minDistance = Infinity;
    let frontClear = true;
    let leftClear = true;
    let rightClear = true;

    const frontAngleThreshold = 30 * Math.PI / 180; // ±30° front sector
    const safeDistance = 10.0; // 10m safe distance

    for (let i = 0; i < ranges.length; i++) {
        const range = ranges[i];
        const angle = angleMin + i * angleIncrement;

        if (range < minDistance && range > 0.5 && range < 100) {
            minDistance = range;
        }

        // Check front sector
        if (Math.abs(angle) < frontAngleThreshold) {
            if (range < safeDistance && range > 0.5) {
                frontClear = false;
            }
        }

        // Check left sector
        if (angle > frontAngleThreshold && angle < Math.PI / 2) {
            if (range < safeDistance && range > 0.5) {
                leftClear = false;
            }
        }

        // Check right sector
        if (angle < -frontAngleThreshold && angle > -Math.PI / 2) {
            if (range < safeDistance && range > 0.5) {
                rightClear = false;
            }
        }
    }

    // Update display
    const minObstacle = document.getElementById('min-obstacle');
    if (minDistance < 100) {
        minObstacle.textContent = minDistance.toFixed(1) + ' m';
        minObstacle.style.color = minDistance < 5.0 ? '#e74c3c' : (minDistance < 10.0 ? '#f39c12' : '#2ecc71');
    } else {
        minObstacle.textContent = 'Clear';
        minObstacle.style.color = '#2ecc71';
    }

    document.getElementById('front-clear').textContent = frontClear ? 'Yes' : 'No';
    document.getElementById('front-clear').style.color = frontClear ? '#2ecc71' : '#e74c3c';

    document.getElementById('left-clear').textContent = leftClear ? 'Yes' : 'No';
    document.getElementById('left-clear').style.color = leftClear ? '#2ecc71' : '#e74c3c';

    document.getElementById('right-clear').textContent = rightClear ? 'Yes' : 'No';
    document.getElementById('right-clear').style.color = rightClear ? '#2ecc71' : '#e74c3c';

    // Update obstacle status badge
    const obstacleStatus = document.getElementById('obstacle-status');
    if (!frontClear || minDistance < 5.0) {
        obstacleStatus.textContent = 'Obstacle Detected';
        obstacleStatus.className = 'value badge danger';
    } else if (minDistance < 10.0) {
        obstacleStatus.textContent = 'Caution';
        obstacleStatus.className = 'value badge warning';
    } else {
        obstacleStatus.textContent = 'Clear';
        obstacleStatus.className = 'value badge clear';
    }
}

function updateConnectionStatus(isConnected) {
    const statusElement = document.getElementById('connection-status');
    if (isConnected) {
        statusElement.textContent = 'Connected';
        statusElement.className = 'status connected';
    } else {
        statusElement.textContent = 'Disconnected';
        statusElement.className = 'status disconnected';
    }
}

function updateMapPosition(latitude, longitude) {
    if (!map || !boatMarker) return;

    const latLng = [latitude, longitude];
    boatMarker.setLatLng(latLng);

    // Add to trajectory
    trajectoryPoints.push(latLng);
    if (trajectoryPoints.length > 300) {
        trajectoryPoints.shift(); // Keep last 300 points
    }
    trajectoryLine.setLatLngs(trajectoryPoints);

    // Center map on boat only if follow mode is enabled
    if (mapFollowBoat) {
        const center = map.getCenter();
        const distance = map.distance(center, latLng);
        if (distance > 50) { // More than 50m from center
            map.panTo(latLng);
        }
    }
}

function addNodeLog(message) {
    const logsContainer = document.getElementById('node-logs');
    const timestamp = new Date().toLocaleTimeString();

    // Map ROS log levels to our classes
    const levelMap = {
        10: 'debug',    // DEBUG
        20: 'info',     // INFO
        30: 'warning',  // WARN
        40: 'error',    // ERROR
        50: 'error'     // FATAL
    };
    const level = levelMap[message.level] || 'info';

    const logEntry = document.createElement('div');
    logEntry.className = 'log-entry ' + level;
    logEntry.innerHTML = `<span class="log-time">[${timestamp}]</span> <span class="log-message">${message.msg}</span>`;

    logsContainer.appendChild(logEntry);

    // Auto-scroll if enabled
    if (document.getElementById('node-logs-auto-scroll').checked) {
        logsContainer.scrollTop = logsContainer.scrollHeight;
    }

    // Keep only last 100 entries
    while (logsContainer.children.length > 100) {
        logsContainer.removeChild(logsContainer.firstChild);
    }
}

function clearNodeLogs() {
    document.getElementById('node-logs').innerHTML = '';
    log('Node logs cleared', 'info');
}

// ============================================================================
// Camera Feed Functions
// ============================================================================
function initCamera() {
    const cameraImage = document.getElementById('camera-image');
    const cameraStatus = document.getElementById('camera-status');

    // Set up event listeners
    cameraImage.addEventListener('load', function() {
        cameraStatus.textContent = 'Streaming';
        cameraStatus.style.display = 'none';
    });

    cameraImage.addEventListener('error', function() {
        cameraStatus.textContent = 'Camera feed unavailable. Make sure web_video_server is running on port 8080';
        cameraStatus.style.display = 'block';
    });

    // Load the camera feed
    updateCameraFeed();
}

function updateCameraFeed() {
    const cameraTopic = document.getElementById('camera-topic').value;
    const cameraImage = document.getElementById('camera-image');
    const cameraStatus = document.getElementById('camera-status');

    cameraStatus.textContent = `Connecting to ${cameraTopic}...`;
    cameraStatus.style.display = 'block';

    // Build URL using current hostname (works for localhost or remote access)
    const baseHost = window.location.hostname || 'localhost';
    const base = `${window.location.protocol}//${baseHost}:8080/stream`;
    const imageUrl = `${base}?topic=${cameraTopic}&type=mjpeg&quality=80`;

    // Cache-bust to force reconnect
    cameraImage.src = `${imageUrl}&t=${Date.now()}`;

    log('Camera feed updated: ' + cameraTopic, 'info');
}

// ============================================================================
// Goal Control Functions
// ============================================================================
function sendGoal() {
    if (!connected) {
        log('Cannot send goal: Not connected to ROS', 'error');
        return;
    }

    const x = parseFloat(document.getElementById('goal-x').value);
    const y = parseFloat(document.getElementById('goal-y').value);

    const goalMessage = new ROSLIB.Message({
        header: {
            frame_id: 'world',
            stamp: {
                sec: 0,
                nanosec: 0
            }
        },
        pose: {
            position: {
                x: x,
                y: y,
                z: 0.0
            },
            orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            }
        }
    });

    goalPublisher.publish(goalMessage);
    log('Goal sent: (' + x + ', ' + y + ')', 'success');
}

function stopAll() {
    if (!connected) {
        log('Cannot stop: Not connected to ROS', 'error');
        return;
    }

    // Publish zero thrust to both thrusters
    const zeroThrust = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/left/thrust',
        messageType: 'std_msgs/Float32'
    });

    zeroThrust.publish(new ROSLIB.Message({ data: 0.0 }));

    const zeroThrustRight = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/right/thrust',
        messageType: 'std_msgs/Float32'
    });

    zeroThrustRight.publish(new ROSLIB.Message({ data: 0.0 }));

    log('EMERGENCY STOP activated', 'error');
}

// ============================================================================
// Logging Functions
// ============================================================================
function log(message, level = 'info') {
    const logsContainer = document.getElementById('logs');
    const timestamp = new Date().toLocaleTimeString();

    const logEntry = document.createElement('div');
    logEntry.className = 'log-entry ' + level;
    logEntry.innerHTML = `<span class="log-time">[${timestamp}]</span> <span class="log-message">${message}</span>`;

    logsContainer.appendChild(logEntry);

    // Auto-scroll if enabled
    if (document.getElementById('logs-auto-scroll').checked) {
        logsContainer.scrollTop = logsContainer.scrollHeight;
    }

    // Keep only last 100 entries
    while (logsContainer.children.length > 100) {
        logsContainer.removeChild(logsContainer.firstChild);
    }
}

function clearLogs() {
    document.getElementById('logs').innerHTML = '';
    log('Logs cleared', 'info');
}

// ============================================================================
// Event Listeners
// ============================================================================
document.addEventListener('DOMContentLoaded', function() {
    // Initialize map
    initMap();

    // Initialize ROS connection
    initROS();

    // Goal control buttons
    document.getElementById('btn-send-goal').addEventListener('click', sendGoal);
    document.getElementById('btn-stop-all').addEventListener('click', stopAll);
    document.getElementById('btn-clear-logs').addEventListener('click', clearLogs);
    document.getElementById('btn-clear-node-logs').addEventListener('click', clearNodeLogs);
    document.getElementById('btn-refresh-camera').addEventListener('click', function() {
        updateCameraFeed();
    });

    // Auto goal sequence buttons (placeholder - needs robust_avoidance node support)
    document.getElementById('btn-enable-auto-goals').addEventListener('click', function() {
        log('Auto goal sequence feature requires parameter update in robust_avoidance node', 'warning');
    });

    document.getElementById('btn-disable-auto-goals').addEventListener('click', function() {
        log('Auto goal sequence feature requires parameter update in robust_avoidance node', 'warning');
    });
});
