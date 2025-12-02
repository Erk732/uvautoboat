/**
 * Atlantis Dashboard - ROS 2 Integration
 * Simple, clean web interface matching Vostok1 style
 */

// ============================================
// ROS CONNECTION
// ============================================
let ros = null;
let connected = false;

// Subscribers
let missionStatusSub = null;
let obstacleStatusSub = null;
let antiStuckSub = null;
let gpsSub = null;
let leftThrustSub = null;
let rightThrustSub = null;
let pathSub = null;

// Publishers
let replanPub = null;
let startPub = null;
let stopPub = null;

// State
let boatPosition = { lat: 0, lon: 0 };
let waypoints = [];
let currentWpIndex = 0;

// Map
let map = null;
let boatMarker = null;
let pathLine = null;
let waypointMarkers = [];

// ============================================
// INITIALIZATION
// ============================================
document.addEventListener('DOMContentLoaded', () => {
    initMap();
    initUI();
    log('Dashboard ready. Click "Connect" to connect to ROS.', 'info');
});

function initMap() {
    map = L.map('map', {
        center: [-33.724, 151.0],
        zoom: 16
    });

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap'
    }).addTo(map);

    const boatIcon = L.divIcon({
        className: 'boat-marker',
        html: '<div style="font-size: 24px;">🚤</div>',
        iconSize: [32, 32],
        iconAnchor: [16, 16]
    });
    boatMarker = L.marker([-33.724, 151.0], { icon: boatIcon }).addTo(map);

    pathLine = L.polyline([], {
        color: '#667eea',
        weight: 3,
        opacity: 0.8,
        dashArray: '10, 5'
    }).addTo(map);
}

function initUI() {
    document.getElementById('btn-connect').addEventListener('click', toggleConnection);
    document.getElementById('btn-start').addEventListener('click', startMission);
    document.getElementById('btn-stop').addEventListener('click', stopMission);
    document.getElementById('btn-replan').addEventListener('click', requestReplan);
    document.getElementById('btn-apply-planner').addEventListener('click', applyPlannerParams);
    document.getElementById('btn-apply-controller').addEventListener('click', applyControllerParams);
    document.getElementById('btn-clear-log').addEventListener('click', clearLog);
}

// ============================================
// ROS CONNECTION
// ============================================
function toggleConnection() {
    if (connected) {
        disconnect();
    } else {
        connect();
    }
}

function connect() {
    const wsUrl = 'ws://localhost:9090';
    log(`Connecting to ${wsUrl}...`, 'info');

    ros = new ROSLIB.Ros({ url: wsUrl });

    ros.on('connection', () => {
        connected = true;
        updateConnectionStatus(true);
        log('Connected to ROS bridge!', 'success');
        setupSubscribers();
        setupPublishers();
    });

    ros.on('error', (error) => {
        log(`Connection error: ${error}`, 'error');
    });

    ros.on('close', () => {
        connected = false;
        updateConnectionStatus(false);
        log('Disconnected from ROS bridge.', 'warn');
    });
}

function disconnect() {
    if (ros) {
        ros.close();
    }
}

function updateConnectionStatus(isConnected) {
    const indicator = document.getElementById('connection-status');
    const btn = document.getElementById('btn-connect');
    
    if (isConnected) {
        indicator.className = 'status connected';
        indicator.textContent = 'Connected';
        btn.textContent = 'Disconnect';
    } else {
        indicator.className = 'status disconnected';
        indicator.textContent = 'Disconnected';
        btn.textContent = 'Connect';
    }
}

// ============================================
// SUBSCRIBERS
// ============================================
function setupSubscribers() {
    missionStatusSub = new ROSLIB.Topic({
        ros: ros,
        name: '/atlantis/mission_status',
        messageType: 'std_msgs/String'
    });
    missionStatusSub.subscribe(handleMissionStatus);

    obstacleStatusSub = new ROSLIB.Topic({
        ros: ros,
        name: '/atlantis/obstacle_status',
        messageType: 'std_msgs/String'
    });
    obstacleStatusSub.subscribe(handleObstacleStatus);

    antiStuckSub = new ROSLIB.Topic({
        ros: ros,
        name: '/atlantis/anti_stuck_status',
        messageType: 'std_msgs/String'
    });
    antiStuckSub.subscribe(handleAntiStuckStatus);

    gpsSub = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/sensors/gps/gps/fix',
        messageType: 'sensor_msgs/NavSatFix'
    });
    gpsSub.subscribe(handleGPS);

    leftThrustSub = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/left/thrust',
        messageType: 'std_msgs/Float64'
    });
    leftThrustSub.subscribe((msg) => updateThruster('left', msg.data));

    rightThrustSub = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/right/thrust',
        messageType: 'std_msgs/Float64'
    });
    rightThrustSub.subscribe((msg) => updateThruster('right', msg.data));

    pathSub = new ROSLIB.Topic({
        ros: ros,
        name: '/atlantis/path',
        messageType: 'nav_msgs/Path'
    });
    pathSub.subscribe(handlePath);
}

// ============================================
// PUBLISHERS
// ============================================
function setupPublishers() {
    replanPub = new ROSLIB.Topic({
        ros: ros,
        name: '/atlantis/replan',
        messageType: 'std_msgs/Empty'
    });

    startPub = new ROSLIB.Topic({
        ros: ros,
        name: '/atlantis/start',
        messageType: 'std_msgs/Empty'
    });

    stopPub = new ROSLIB.Topic({
        ros: ros,
        name: '/atlantis/stop',
        messageType: 'std_msgs/Empty'
    });
}

// ============================================
// MESSAGE HANDLERS
// ============================================
function handleMissionStatus(msg) {
    try {
        const data = JSON.parse(msg.data);
        updateMissionState(data.state);
        document.getElementById('current-waypoint').textContent = `${data.waypoint} / ${data.total_waypoints}`;
        document.getElementById('distance-to-wp').textContent = `${data.distance_to_waypoint.toFixed(1)} m`;
        
        if (data.local_x !== undefined) {
            document.getElementById('local-x').textContent = `${data.local_x.toFixed(1)} m`;
            document.getElementById('local-y').textContent = `${data.local_y.toFixed(1)} m`;
            document.getElementById('local-yaw').textContent = `${data.yaw_deg.toFixed(1)}°`;
        }
    } catch (e) {
        console.error('Error parsing mission status:', e);
    }
}

function handleObstacleStatus(msg) {
    try {
        const data = JSON.parse(msg.data);
        const badge = document.getElementById('obstacle-badge');
        
        if (data.min_distance < 5) {
            badge.className = 'value badge danger';
            badge.textContent = 'DANGER';
        } else if (data.min_distance < 15) {
            badge.className = 'value badge warning';
            badge.textContent = 'WARNING';
        } else {
            badge.className = 'value badge clear';
            badge.textContent = 'CLEAR';
        }
        
        document.getElementById('min-obstacle-dist').textContent = `${data.min_distance.toFixed(1)} m`;
        document.getElementById('sector-front').textContent = data.front_clear ? '✓' : '✗';
        document.getElementById('sector-left').textContent = data.left_clear ? '✓' : '✗';
        document.getElementById('sector-right').textContent = data.right_clear ? '✓' : '✗';
    } catch (e) {
        console.error('Error parsing obstacle status:', e);
    }
}

function handleAntiStuckStatus(msg) {
    try {
        const data = JSON.parse(msg.data);
        const badge = document.getElementById('sass-badge');
        
        if (data.escape_mode) {
            badge.className = 'value badge escaping';
            badge.textContent = 'ESCAPING';
        } else if (data.is_stuck) {
            badge.className = 'value badge stuck';
            badge.textContent = 'STUCK';
        } else {
            badge.className = 'value badge clear';
            badge.textContent = 'NORMAL';
        }
        
        document.getElementById('escape-phase').textContent = data.escape_phase || 'IDLE';
        document.getElementById('escape-direction').textContent = data.best_direction || '--';
        document.getElementById('escape-attempts').textContent = data.consecutive_attempts || 0;
        document.getElementById('no-go-zones').textContent = data.no_go_zones || 0;
    } catch (e) {
        console.error('Error parsing anti-stuck status:', e);
    }
}

function handleGPS(msg) {
    boatPosition.lat = msg.latitude;
    boatPosition.lon = msg.longitude;
    
    document.getElementById('gps-status').textContent = `${msg.latitude.toFixed(6)}, ${msg.longitude.toFixed(6)}`;
    
    boatMarker.setLatLng([msg.latitude, msg.longitude]);
}

function handlePath(msg) {
    waypoints = msg.poses.map(p => ({
        x: p.pose.position.x,
        y: p.pose.position.y
    }));
    
    log(`Received path with ${waypoints.length} waypoints`, 'info');
}

function updateMissionState(state) {
    const badge = document.getElementById('mission-state');
    const stateMap = {
        'WAITING_FOR_PATH': { class: 'idle', text: 'WAITING' },
        'DRIVING': { class: 'driving', text: 'DRIVING' },
        'MOVING_TO_WAYPOINT': { class: 'driving', text: 'MOVING' },
        'OBSTACLE_AVOIDING': { class: 'avoiding', text: 'AVOIDING' },
        'STUCK_ESCAPING': { class: 'stuck', text: 'ESCAPING' },
        'MISSION_COMPLETE': { class: 'finished', text: 'COMPLETE' },
        'FINISHED': { class: 'finished', text: 'FINISHED' },
        'PAUSED': { class: 'paused', text: 'PAUSED' }
    };
    
    const s = stateMap[state] || { class: 'idle', text: state };
    badge.className = `value badge ${s.class}`;
    badge.textContent = s.text;
}

function updateThruster(side, value) {
    const bar = document.getElementById(`${side}-bar`);
    const valueEl = document.getElementById(`${side}-value`);
    const maxThrust = 1000;
    
    const percentage = Math.abs(value) / maxThrust * 50;
    bar.style.width = `${percentage}%`;
    bar.className = 'thrust-fill';
    
    if (value >= 0) {
        bar.classList.add('forward');
        bar.style.left = '50%';
        bar.style.right = 'auto';
    } else {
        bar.classList.add('reverse');
        bar.style.left = 'auto';
        bar.style.right = '50%';
    }
    
    valueEl.textContent = `${value.toFixed(0)} N`;
}

// ============================================
// ACTIONS
// ============================================
function startMission() {
    if (!connected) {
        log('Not connected to ROS!', 'error');
        return;
    }
    log('Starting mission...', 'info');
    startPub.publish(new ROSLIB.Message({}));
    setTimeout(requestReplan, 200);
}

function stopMission() {
    if (!connected) {
        log('Not connected to ROS!', 'error');
        return;
    }
    log('Stopping mission...', 'warn');
    stopPub.publish(new ROSLIB.Message({}));
}

function requestReplan() {
    if (!connected) {
        log('Not connected to ROS!', 'error');
        return;
    }
    log('Requesting path replan...', 'info');
    replanPub.publish(new ROSLIB.Message({}));
}

// ============================================
// PARAMETER FUNCTIONS
// ============================================
function applyPlannerParams() {
    const params = {
        lanes: parseInt(document.getElementById('param-lanes').value),
        scan_length: parseFloat(document.getElementById('param-scan-length').value),
        scan_width: parseFloat(document.getElementById('param-scan-width').value)
    };
    
    setROS2Parameters('atlantis_planner', params);
    log(`Planner params: lanes=${params.lanes}, length=${params.scan_length}m, width=${params.scan_width}m`, 'success');
    setTimeout(requestReplan, 500);
}

function applyControllerParams() {
    const params = {
        kp: parseFloat(document.getElementById('param-kp').value),
        ki: parseFloat(document.getElementById('param-ki').value),
        kd: parseFloat(document.getElementById('param-kd').value),
        base_speed: parseFloat(document.getElementById('param-base-speed').value),
        max_speed: parseFloat(document.getElementById('param-max-speed').value),
        min_safe_distance: parseFloat(document.getElementById('param-safe-dist').value)
    };
    
    setROS2Parameters('atlantis_controller', params);
    log(`Controller params: Kp=${params.kp}, Ki=${params.ki}, Kd=${params.kd}`, 'success');
}

function setROS2Parameters(nodeName, params) {
    if (!connected) {
        log('Not connected to ROS!', 'error');
        return;
    }
    
    const paramService = new ROSLIB.Service({
        ros: ros,
        name: `/${nodeName}/set_parameters`,
        serviceType: 'rcl_interfaces/srv/SetParameters'
    });
    
    const parameterList = Object.entries(params).map(([name, value]) => {
        let paramValue = {};
        if (Number.isInteger(value)) {
            paramValue = { type: 2, integer_value: value };
        } else {
            paramValue = { type: 3, double_value: value };
        }
        return { name, value: paramValue };
    });
    
    paramService.callService(
        { parameters: parameterList },
        (result) => console.log('Parameters set:', result),
        (error) => log(`Failed to set parameters: ${error}`, 'error')
    );
}

// ============================================
// LOGGING
// ============================================
function log(message, level = 'info') {
    const logContainer = document.getElementById('log-output');
    const entry = document.createElement('div');
    const timestamp = new Date().toLocaleTimeString();
    entry.className = `log-entry ${level}`;
    entry.textContent = `[${timestamp}] ${message}`;
    logContainer.appendChild(entry);
    
    if (document.getElementById('auto-scroll').checked) {
        logContainer.scrollTop = logContainer.scrollHeight;
    }
}

function clearLog() {
    const logContainer = document.getElementById('log-output');
    logContainer.innerHTML = '<div class="log-entry info">[INFO] Log cleared.</div>';
}
