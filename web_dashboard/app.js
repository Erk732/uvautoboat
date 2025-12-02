// ROS Connection
let ros;
let connected = false;

// Map variables
let map;
let boatMarker;
let trajectoryLine;
let trajectoryPoints = [];
let waypointMarkers = [];
let waypointPath = null;  // Line connecting waypoints
let currentWaypointMarker = null;  // Highlight current target

// Configuration publisher
let configPublisher = null;
let modularConfigPublisher = null;  // For sputnik planner
let missionCommandPublisher = null;  // Mission command publisher

// Track which config inputs have been modified by user (prevents ROS from overwriting)
let dirtyInputs = new Set();

// Mission state
let missionState = {
    state: 'IDLE',
    gpsReady: false,
    waypointsGenerated: false,
    missionArmed: false,
    joystickOverride: false,
    waypoints: [],
    currentWaypoint: 0,
    totalWaypoints: 0,
    startLat: null,
    startLon: null
};

// Data storage
let currentState = {
    gps: { lat: 0, lon: 0, local_x: 0, local_y: 0 },
    obstacles: { min: Infinity, front: true, left: true, right: true },
    thrusters: { left: 0, right: 0 },
    mission: { state: 'IDLE', waypoint: 0, distance: 0 },
    config: {
        scan_length: 150.0,
        scan_width: 50.0,
        lanes: 8,
        kp: 400.0,
        ki: 20.0,
        kd: 100.0,
        base_speed: 500.0,
        max_speed: 800.0,
        min_safe_distance: 15.0
    }
};

// Style mode state: 'normal', 'bureau', 'terminal'
let currentStyleMode = 'normal';
const styleModes = ['normal', 'bureau', 'terminal'];
const styleModeLabels = {
    'normal': 'БЮРО | BUREAU MODE',
    'bureau': 'ТЕРМИНАЛ | TERMINAL MODE', 
    'terminal': 'ОБЫЧНЫЙ | NORMAL MODE'
};
const styleModeLogMessages = {
    'normal': 'Переключено на обычный режим | Switched to normal mode',
    'bureau': 'Переключено на режим бюро | Switched to bureau mode',
    'terminal': 'Переключено на режим терминала | Switched to terminal mode'
};

// Initialize everything when page loads
window.addEventListener('load', () => {
    console.log('Dashboard loading...');
    initMap();
    connectToROS();
    initStyleToggle();
    initConfigPanel();
    initMissionControl();  // NEW: Mission control buttons
    initTerminal();
    addLog('Dashboard initialized', 'info');
});

// Initialize style toggle button - cycles through 3 modes
function initStyleToggle() {
    const toggleBtn = document.getElementById('style-toggle');
    const toggleText = document.getElementById('toggle-text');
    const body = document.body;
    const container = document.querySelector('.container');
    
    toggleBtn.addEventListener('click', () => {
        // Remove current style class
        body.classList.remove('bureau-mode', 'terminal-mode');
        container.classList.remove('bureau-mode', 'terminal-mode');
        
        // Cycle to next style
        const currentIndex = styleModes.indexOf(currentStyleMode);
        const nextIndex = (currentIndex + 1) % styleModes.length;
        currentStyleMode = styleModes[nextIndex];
        
        // Apply new style class (normal has no class)
        if (currentStyleMode !== 'normal') {
            body.classList.add(`${currentStyleMode}-mode`);
            container.classList.add(`${currentStyleMode}-mode`);
        }
        
        // Update button text to show NEXT mode
        toggleText.textContent = styleModeLabels[currentStyleMode];
        
        // Log the change
        addLog(styleModeLogMessages[currentStyleMode], 'info');
        console.log(`Style changed to: ${currentStyleMode}`);
    });
    
    console.log('Style toggle initialized (3-mode cycle: Normal → Bureau → Terminal)');
}


// Map follow mode
let mapFollowBoat = false;  // Default: don't auto-follow

// Update follow button visual state
function updateFollowButtonState() {
    const btn = document.getElementById('follow-boat-toggle');
    if (btn) {
        if (mapFollowBoat) {
            btn.innerHTML = '🔒';
            btn.classList.add('active');
            btn.title = 'Following Boat (Click to unlock) | Слежение за судном (Нажмите для отключения)';
        } else {
            btn.innerHTML = '🎯';
            btn.classList.remove('active');
            btn.title = 'Click to follow boat | Нажмите для слежения';
        }
    }
}

// Initialize Leaflet map
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
            button.title = 'Follow Boat | Следить за судном';
            button.innerHTML = '🎯';
            button.id = 'follow-boat-toggle';
            
            L.DomEvent.on(button, 'click', function(e) {
                L.DomEvent.preventDefault(e);
                L.DomEvent.stopPropagation(e);
                mapFollowBoat = !mapFollowBoat;
                updateFollowButtonState();
                if (mapFollowBoat && currentState.gps.lat !== 0) {
                    map.panTo([currentState.gps.lat, currentState.gps.lon]);
                }
            });
            
            return container;
        }
    });
    
    map.addControl(new FollowBoatControl());
    updateFollowButtonState();
    
    addLog('Map initialized', 'info');
}

// Connect to ROS via rosbridge
function connectToROS() {
    console.log('Attempting to connect to ws://localhost:9090...');
    ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    ros.on('connection', () => {
        console.log('Connected to rosbridge!');
        connected = true;
        updateConnectionStatus(true);
        addLog('Connected to rosbridge', 'info');
        subscribeToTopics();
    });

    ros.on('error', (error) => {
        console.error('ROS connection error:', error);
        connected = false;
        updateConnectionStatus(false);
        addLog(`Connection error: ${error}`, 'error');
    });

    ros.on('close', () => {
        console.log('Connection closed');
        connected = false;
        updateConnectionStatus(false);
        addLog('Connection closed. Retrying in 5s...', 'warning');
        setTimeout(connectToROS, 5000);
    });
}

// Update connection status indicator
function updateConnectionStatus(isConnected) {
    const statusElement = document.getElementById('connection-status');
    if (isConnected) {
        statusElement.textContent = 'Подключено | Connected';
        statusElement.className = 'status connected';
    } else {
        statusElement.textContent = 'Отключено | Disconnected';
        statusElement.className = 'status disconnected';
    }
}

// Subscribe to ROS topics
function subscribeToTopics() {
    console.log('Subscribing to topics...');
    
    // GPS Fix
    const gpsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/sensors/gps/gps/fix',
        messageType: 'sensor_msgs/NavSatFix'
    });

    gpsTopic.subscribe((message) => {
        console.log('GPS data received:', message.latitude, message.longitude);
        updateGPS(message);
    });

    // Left thruster
    const leftThrustTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/left/thrust',
        messageType: 'std_msgs/Float64'
    });

    leftThrustTopic.subscribe((message) => {
        console.log('Left thrust:', message.data);
        updateThruster('left', message.data);
    });

    // Right thruster
    const rightThrustTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/right/thrust',
        messageType: 'std_msgs/Float64'
    });

    rightThrustTopic.subscribe((message) => {
        console.log('Right thrust:', message.data);
        updateThruster('right', message.data);
    });
    
    // Mission status
    const missionStatusTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/mission_status',
        messageType: 'std_msgs/String'
    });
    
    missionStatusTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Mission status:', data);
        updateMissionStatus(data);
    });
    
    // Obstacle status
    const obstacleStatusTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/obstacle_status',
        messageType: 'std_msgs/String'
    });
    
    obstacleStatusTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Obstacle status:', data);
        updateObstacleStatus(data);
    });
    
    // Smart Anti-Stuck Status (SASS)
    const antiStuckTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/anti_stuck_status',
        messageType: 'std_msgs/String'
    });
    
    antiStuckTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Anti-stuck status:', data);
        updateAntiStuckStatus(data);
    });
    
    // Also subscribe to modular controller anti-stuck (for modular mode)
    const buranAntiStuckTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/control/anti_stuck_status',
        messageType: 'std_msgs/String'
    });
    
    buranAntiStuckTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Buran anti-stuck status:', data);
        updateAntiStuckStatus(data);
    });

    // ==================== MODULAR NAVIGATION SUPPORT ====================
    // Subscribe to modular navigation topics (sputnik_planner, oko_perception, buran_controller)
    
    // Modular mission status from sputnik_planner
    const modularMissionTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/planning/mission_status',
        messageType: 'std_msgs/String'
    });
    
    modularMissionTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Modular mission status:', data);
        // Convert modular format to vostok1 format
        // Sputnik uses: current_waypoint, total_waypoints, progress_percent, elapsed_time, position
        updateMissionStatus({
            state: data.state,
            waypoint: data.current_waypoint,
            total_waypoints: data.total_waypoints,
            distance_to_waypoint: data.distance_to_target || 0,
            // Add local position from sputnik's position array
            local_x: data.position ? data.position[0] : 0,
            local_y: data.position ? data.position[1] : 0
        });
    });
    
    // Modular current target from sputnik_planner (for distance info)
    const modularTargetTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/planning/current_target',
        messageType: 'std_msgs/String'
    });
    
    modularTargetTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Modular current target:', data);
        // Update distance to waypoint display
        if (data.distance_to_target !== undefined) {
            document.getElementById('distance').textContent = data.distance_to_target.toFixed(1) + 'm';
        }
    });
    
    // Modular waypoints from sputnik_planner
    const modularWaypointsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/planning/waypoints',
        messageType: 'std_msgs/String'
    });
    
    modularWaypointsTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Modular waypoints received:', data);
        if (data.waypoints && data.waypoints.length > 0) {
            // Check if waypoints actually changed
            const newWpString = JSON.stringify(data.waypoints);
            const oldWpString = JSON.stringify(missionState.waypoints);
            const waypointsChanged = newWpString !== oldWpString;
            
            missionState.waypoints = data.waypoints;
            missionState.totalWaypoints = data.total || data.waypoints.length;
            displayWaypointsOnMap(data.waypoints, waypointsChanged);
        }
    });
    
    // Modular obstacle status from oko_perception
    const modularObstacleTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/perception/obstacle_info',
        messageType: 'std_msgs/String'
    });
    
    modularObstacleTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Modular obstacle status:', data);
        // Convert modular format to vostok1 format
        updateObstacleStatus({
            min_distance: data.min_distance,
            front_clear: data.front_clear > 15,
            left_clear: data.left_clear > 15,
            right_clear: data.right_clear > 15,
            front_distance: data.front_clear,
            left_distance: data.left_clear,
            right_distance: data.right_clear,
            status: data.is_critical ? '🚨 КРИТИЧНО | CRITICAL' : 
                    data.obstacle_detected ? '⚠️ ПРЕПЯТСТВИЕ | OBSTACLE' : 
                    '✅ СВОБОДНО | CLEAR'
        });
    });
    
    // Modular controller status from buran_controller
    const modularControlTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/control/status',
        messageType: 'std_msgs/String'
    });
    
    modularControlTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Modular control status:', data);
        // Can be used for additional status display if needed
    });

    console.log('Subscribed to all topics (integrated + modular)');
    addLog('Subscribed to topics (Vostok1 + Modular)', 'info');
    
    // Create publisher for configuration updates (supports both modes)
    configPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/set_config',
        messageType: 'std_msgs/String'
    });
    
    // Also create modular config publisher for sputnik
    modularConfigPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/sputnik/set_config',
        messageType: 'std_msgs/String'
    });
    
    // Subscribe to current config (vostok1)
    const configTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/config',
        messageType: 'std_msgs/String'
    });
    
    configTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Config received (vostok1):', data);
        updateConfigFromROS(data);
    });
    
    // Subscribe to current config (modular sputnik)
    const modularConfigTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/sputnik/config',
        messageType: 'std_msgs/String'
    });
    
    modularConfigTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Config received (sputnik):', data);
        updateConfigFromROS(data);
    });
    
    // Subscribe to waypoints for map preview (vostok1)
    const waypointsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/waypoints',
        messageType: 'std_msgs/String'
    });
    
    waypointsTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        if (data.waypoints && data.waypoints.length > 0) {
            // Check if waypoints actually changed
            const newWpString = JSON.stringify(data.waypoints);
            const oldWpString = JSON.stringify(missionState.waypoints);
            const waypointsChanged = newWpString !== oldWpString;
            
            missionState.waypoints = data.waypoints;
            displayWaypointsOnMap(data.waypoints, waypointsChanged);
        }
    });
    
    // Create mission command publisher (supports both modes)
    missionCommandPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/mission_command',
        messageType: 'std_msgs/String'
    });
    
    // Subscribe to ROS logs (rosout)
    const rosoutTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/rosout',
        messageType: 'rcl_interfaces/Log'
    });
    
    rosoutTopic.subscribe((message) => {
        // Filter for vostok1 and modular node messages
        if (message.name && (
            message.name.includes('vostok1') ||
            message.name.includes('sputnik') ||
            message.name.includes('oko') ||
            message.name.includes('buran')
        )) {
            addTerminalLine(message);
        }
    });
    
    addTerminalLine({ level: 20, msg: 'Подключено к ROS | Connected to ROS', name: 'system' });
}

// Update GPS data
function updateGPS(message) {
    currentState.gps.lat = message.latitude;
    currentState.gps.lon = message.longitude;
    
    // Update display
    document.getElementById('latitude').textContent = message.latitude.toFixed(6) + '°';
    document.getElementById('longitude').textContent = message.longitude.toFixed(6) + '°';
    
    // Update map
    const latLng = [message.latitude, message.longitude];
    boatMarker.setLatLng(latLng);
    
    // Add to trajectory
    trajectoryPoints.push(latLng);
    if (trajectoryPoints.length > 100) {
        trajectoryPoints.shift(); // Keep last 100 points
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

// Update thruster data
function updateThruster(side, value) {
    currentState.thrusters[side] = value;
    
    // Update display
    document.getElementById(`${side}-thrust`).textContent = value.toFixed(1);
    
    // Update thrust bar
    const bar = document.getElementById(`${side}-thrust-bar`);
    const percentage = Math.min(Math.abs(value) / 1000 * 100, 100);
    bar.style.width = percentage + '%';
    
    // Color based on direction
    if (value < 0) {
        bar.classList.add('reverse');
    } else {
        bar.classList.remove('reverse');
    }
}

// Simulate mission data (would come from custom topics in real implementation)
function updateMissionStatus(data) {
    currentState.mission.state = data.state;
    currentState.mission.waypoint = data.waypoint;
    currentState.mission.distance = data.distance_to_waypoint;
    
    document.getElementById('state').textContent = data.state.replace(/_/g, ' ');
    document.getElementById('waypoint').textContent = `${data.waypoint}/${data.total_waypoints}`;
    document.getElementById('distance').textContent = data.distance_to_waypoint.toFixed(1) + 'm';
}

// Simulate obstacle data (would parse from PointCloud2 or custom topic)
function updateObstacleStatus(data) {
    currentState.obstacles.min = data.min_distance;
    currentState.obstacles.front = data.front_clear;
    currentState.obstacles.left = data.left_clear;
    currentState.obstacles.right = data.right_clear;
    
    const minDist = data.min_distance;
    document.getElementById('min-obstacle').textContent = 
        minDist >= 999 ? '∞' : minDist.toFixed(1) + 'm';
    
    document.getElementById('front-clear').textContent = 
        data.front_clear ? `✓ ${data.front_distance}m` : '✗ Blocked';
    document.getElementById('left-clear').textContent = 
        data.left_clear ? `✓ ${data.left_distance}m` : `✗ ${data.left_distance}m`;
    document.getElementById('right-clear').textContent = 
        data.right_clear ? `✓ ${data.right_distance}m` : `✗ ${data.right_distance}m`;
    
    // Update status badge with bilingual message from vostok1
    const statusBadge = document.getElementById('obstacle-status');
    if (data.status) {
        // Use the bilingual status text from vostok1.py
        statusBadge.textContent = data.status;
        
        // Set class based on emoji indicator
        if (data.status.includes('✅')) {
            statusBadge.className = 'value badge clear';
        } else if (data.status.includes('🚨')) {
            statusBadge.className = 'value badge critical';
        } else {
            statusBadge.className = 'value badge warning';
        }
    } else {
        // Fallback to old behavior if status not provided
        if (minDist > 15 || minDist >= 999) {
            statusBadge.textContent = 'Свободно | Clear';
            statusBadge.className = 'value badge clear';
        } else if (minDist > 5) {
            statusBadge.textContent = 'Внимание | Warning';
            statusBadge.className = 'value badge warning';
        } else {
            statusBadge.textContent = 'Критично | Critical';
            statusBadge.className = 'value badge critical';
        }
    }
}

// Update Smart Anti-Stuck Status (SASS)
function updateAntiStuckStatus(data) {
    // Update anti-stuck panel elements if they exist
    const stuckStatus = document.getElementById('stuck-status');
    const escapePhase = document.getElementById('escape-phase');
    const noGoZones = document.getElementById('no-go-zones');
    const driftVector = document.getElementById('drift-vector');
    const escapeHistory = document.getElementById('escape-history');
    const probeResults = document.getElementById('probe-results');
    
    if (stuckStatus) {
        if (data.is_stuck && data.escape_mode) {
            stuckStatus.textContent = `ЗАСТРЯЛ | STUCK (Попытка ${data.consecutive_attempts})`;
            stuckStatus.className = 'value badge critical';
        } else {
            stuckStatus.textContent = 'Норма | Normal';
            stuckStatus.className = 'value badge clear';
        }
    }
    
    if (escapePhase) {
        const phases = ['PROBE', 'REVERSE', 'TURN', 'FORWARD', 'IDLE'];
        const phaseNames = ['ЗОНДИРОВАНИЕ', 'РЕВЕРС', 'ПОВОРОТ', 'ВПЕРЕД', 'ОЖИДАНИЕ'];
        const idx = data.escape_mode ? data.escape_phase : 4;
        escapePhase.textContent = `${phaseNames[idx]} | ${phases[idx]}`;
        escapePhase.className = data.escape_mode ? 'value active' : 'value';
    }
    
    if (noGoZones) {
        noGoZones.textContent = `${data.no_go_zones} зон | zones`;
    }
    
    if (driftVector) {
        const dx = data.drift_vector[0];
        const dy = data.drift_vector[1];
        const magnitude = Math.hypot(dx, dy);
        if (magnitude > 0.1) {
            const angle = Math.atan2(dy, dx) * 180 / Math.PI;
            driftVector.textContent = `${magnitude.toFixed(2)} m/s @ ${angle.toFixed(0)}°`;
        } else {
            driftVector.textContent = 'Минимально | Minimal';
        }
    }
    
    // Display Kalman filter uncertainty
    const driftUncertainty = document.getElementById('drift-uncertainty');
    if (driftUncertainty && data.drift_uncertainty) {
        const ux = data.drift_uncertainty[0];
        const uy = data.drift_uncertainty[1];
        const avgUncertainty = Math.hypot(ux, uy);
        if (avgUncertainty < 0.1) {
            driftUncertainty.textContent = `Высокая | High conf.`;
            driftUncertainty.style.color = '#4CAF50';
        } else if (avgUncertainty < 0.5) {
            driftUncertainty.textContent = `σ=${avgUncertainty.toFixed(2)}`;
            driftUncertainty.style.color = '#FFC107';
        } else {
            driftUncertainty.textContent = `σ=${avgUncertainty.toFixed(2)} (сходится)`;
            driftUncertainty.style.color = '#FF9800';
        }
    }
    
    if (escapeHistory) {
        escapeHistory.textContent = `${data.escape_history_count} записей | records`;
    }
    
    if (probeResults && data.probe_results) {
        probeResults.textContent = `L:${data.probe_results.left}m R:${data.probe_results.right}m`;
    }
    
    // Update best direction indicator if exists
    const bestDirection = document.getElementById('best-direction');
    if (bestDirection && data.best_direction) {
        bestDirection.textContent = data.best_direction === 'LEFT' ? '← ЛЕВО | LEFT' : '→ ПРАВО | RIGHT';
    }
    
    // Add terminal log for stuck events
    if (data.is_stuck && data.escape_mode) {
        addTerminalLine({
            level: 30,  // WARN
            msg: `SASS: Phase ${data.escape_phase} | Duration: ${data.adaptive_duration}s | Zones: ${data.no_go_zones}`,
            name: 'anti_stuck'
        });
    }
}

// Add log entry
function addLog(message, type = 'info') {
    const logsContainer = document.getElementById('logs');
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry ${type}`;
    
    const timestamp = new Date().toLocaleTimeString();
    logEntry.innerHTML = `
        <span class="timestamp">[${timestamp}]</span>
        <span class="message">${message}</span>
    `;
    
    logsContainer.insertBefore(logEntry, logsContainer.firstChild);
    
    // Keep only last 50 logs
    while (logsContainer.children.length > 50) {
        logsContainer.removeChild(logsContainer.lastChild);
    }
}

// Convert GPS to local coordinates (simplified)
function gpsToLocal(lat, lon) {
    // This is a simplified conversion - vostok1 uses proper UTM conversion
    // For display purposes only
    const originLat = -33.8361;
    const originLon = 151.0697;
    
    const latDiff = (lat - originLat) * 111320; // meters per degree latitude
    const lonDiff = (lon - originLon) * 111320 * Math.cos(originLat * Math.PI / 180);
    
    return { x: lonDiff, y: latDiff };
}

// Update local coordinates display
setInterval(() => {
    const local = gpsToLocal(currentState.gps.lat, currentState.gps.lon);
    document.getElementById('local-x').textContent = local.x.toFixed(1) + 'm';
    document.getElementById('local-y').textContent = local.y.toFixed(1) + 'm';
}, 1000);

// Initialize configuration panel
function initConfigPanel() {
    console.log('Initializing config panel...');
    
    // All config input IDs
    const allConfigInputs = [
        'cfg-lanes', 'cfg-scan-length', 'cfg-scan-width',
        'cfg-kp', 'cfg-ki', 'cfg-kd',
        'cfg-base-speed', 'cfg-max-speed', 'cfg-safe-dist',
        'wp-lanes', 'wp-length', 'wp-width'
    ];
    
    // Mark input as dirty when user types
    allConfigInputs.forEach(id => {
        const el = document.getElementById(id);
        if (el) {
            el.addEventListener('input', () => {
                dirtyInputs.add(id);
                el.classList.add('input-dirty');
            });
            // Also mark dirty on focus (user intends to edit)
            el.addEventListener('focus', () => {
                dirtyInputs.add(id);
                el.classList.add('input-dirty');
            });
        }
    });
    
    // Apply all config button (now applies PID/Speed only)
    document.getElementById('btn-apply-config').addEventListener('click', () => {
        sendConfig(true, false);  // PID only
        // Don't clear dirty state immediately - wait for ROS to confirm
        // The dirty state will be cleared when we receive matching values from ROS
        // For now, just give feedback
        addLog('Config sent - waiting for confirmation...', 'info');
    });
    
    console.log('Config panel initialized');
}

// Clear dirty state for specified inputs
function clearDirtyInputs(inputIds) {
    inputIds.forEach(id => {
        dirtyInputs.delete(id);
        const el = document.getElementById(id);
        if (el) el.classList.remove('input-dirty');
    });
}

// Update config inputs from ROS
function updateConfigFromROS(data) {
    currentState.config = { ...currentState.config, ...data };
    
    // Only update inputs if they're not dirty (user hasn't modified them)
    const inputs = {
        'cfg-lanes': data.lanes,
        'cfg-scan-length': data.scan_length,
        'cfg-scan-width': data.scan_width,
        'cfg-kp': data.kp,
        'cfg-ki': data.ki,
        'cfg-kd': data.kd,
        'cfg-base-speed': data.base_speed,
        'cfg-max-speed': data.max_speed,
        'cfg-safe-dist': data.min_safe_distance
    };
    
    // Also update mission control inputs
    const wpInputs = {
        'wp-lanes': data.lanes,
        'wp-length': data.scan_length,
        'wp-width': data.scan_width
    };
    
    for (const [id, value] of Object.entries(inputs)) {
        const el = document.getElementById(id);
        // Don't update if input is dirty (user modified) or focused
        if (el && !dirtyInputs.has(id) && document.activeElement !== el && value !== undefined) {
            el.value = value;
        }
        // Clear dirty state if ROS value matches what we sent (confirmation)
        if (el && dirtyInputs.has(id) && parseFloat(el.value) === value) {
            dirtyInputs.delete(id);
            el.classList.remove('input-dirty');
        }
    }
    
    for (const [id, value] of Object.entries(wpInputs)) {
        const el = document.getElementById(id);
        // Don't update if input is dirty (user modified) or focused
        if (el && !dirtyInputs.has(id) && document.activeElement !== el && value !== undefined) {
            el.value = value;
        }
        // Clear dirty state if ROS value matches what we sent (confirmation)
        if (el && dirtyInputs.has(id) && parseFloat(el.value) === value) {
            dirtyInputs.delete(id);
            el.classList.remove('input-dirty');
        }
    }
    
    // Update mission control UI
    updateMissionControlUI(data);
}

// Send configuration to Vostok1
function sendConfig(pidOnly = false, restart = false) {
    if (!connected || !configPublisher) {
        addLog('Not connected to ROS', 'error');
        return;
    }
    
    let config = {};
    
    if (pidOnly) {
        // Only send PID parameters
        config = {
            kp: parseFloat(document.getElementById('cfg-kp').value),
            ki: parseFloat(document.getElementById('cfg-ki').value),
            kd: parseFloat(document.getElementById('cfg-kd').value)
        };
        addLog('Отправка ПИД | Sending PID config...', 'info');
    } else {
        // Send all parameters
        config = {
            lanes: parseInt(document.getElementById('cfg-lanes').value),
            scan_length: parseFloat(document.getElementById('cfg-scan-length').value),
            scan_width: parseFloat(document.getElementById('cfg-scan-width').value),
            kp: parseFloat(document.getElementById('cfg-kp').value),
            ki: parseFloat(document.getElementById('cfg-ki').value),
            kd: parseFloat(document.getElementById('cfg-kd').value),
            base_speed: parseFloat(document.getElementById('cfg-base-speed').value),
            max_speed: parseFloat(document.getElementById('cfg-max-speed').value),
            min_safe_distance: parseFloat(document.getElementById('cfg-safe-dist').value)
        };
        addLog('Отправка конфигурации | Sending full config...', 'info');
    }
    
    if (restart) {
        config.restart_mission = true;
        addLog('Перезапуск миссии | Restarting mission...', 'warning');
    }
    
    const message = new ROSLIB.Message({
        data: JSON.stringify(config)
    });
    
    // Publish to both vostok1 and modular (sputnik) topics
    configPublisher.publish(message);
    if (modularConfigPublisher) {
        modularConfigPublisher.publish(message);
    }
    addLog('Конфигурация отправлена | Config sent!', 'info');
    console.log('Config sent to both vostok1 and sputnik:', config);
}

// ========== TERMINAL OUTPUT FUNCTIONS ==========

// Add line to terminal output
function addTerminalLine(message) {
    const terminal = document.getElementById('terminal-output');
    if (!terminal) return;
    
    const line = document.createElement('div');
    line.className = 'terminal-line';
    
    // Determine log level class
    // ROS log levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
    let levelClass = 'info';
    let levelPrefix = '[INFO]';
    
    if (message.level !== undefined) {
        if (message.level >= 40) {
            levelClass = 'error';
            levelPrefix = '[ERROR]';
        } else if (message.level >= 30) {
            levelClass = 'warn';
            levelPrefix = '[WARN]';
        } else if (message.level <= 10) {
            levelClass = 'debug';
            levelPrefix = '[DEBUG]';
        }
    }
    
    if (message.name === 'system') {
        levelClass = 'system';
        levelPrefix = '[SYSTEM]';
    }
    
    line.classList.add(levelClass);
    
    // Format timestamp
    const now = new Date();
    const timestamp = now.toLocaleTimeString('en-US', { hour12: false });
    
    // Format the message
    const msgText = message.msg || message;
    line.innerHTML = `<span class="timestamp">[${timestamp}]</span> <span class="level">${levelPrefix}</span> <span class="message">${msgText}</span>`;
    
    terminal.appendChild(line);
    
    // Auto-scroll if enabled
    const autoScroll = document.getElementById('auto-scroll');
    if (autoScroll && autoScroll.checked) {
        terminal.scrollTop = terminal.scrollHeight;
    }
    
    // Limit lines to prevent memory issues
    while (terminal.children.length > 200) {
        terminal.removeChild(terminal.firstChild);
    }
}

// Initialize terminal controls
function initTerminal() {
    const clearBtn = document.getElementById('btn-clear-terminal');
    if (clearBtn) {
        clearBtn.addEventListener('click', () => {
            const terminal = document.getElementById('terminal-output');
            terminal.innerHTML = '<div class="terminal-line system">[SYSTEM] Терминал очищен | Terminal cleared</div>';
        });
    }
}

// ========== MISSION CONTROL FUNCTIONS ==========

// Initialize mission control panel
function initMissionControl() {
    console.log('Initializing mission control...');
    
    // Step 1: Generate Waypoints
    document.getElementById('btn-generate-waypoints').addEventListener('click', () => {
        generateWaypoints();
    });
    
    // Step 2: Confirm/Cancel Waypoints
    document.getElementById('btn-confirm-waypoints').addEventListener('click', () => {
        sendMissionCommand('confirm_waypoints');
    });
    
    document.getElementById('btn-cancel-waypoints').addEventListener('click', () => {
        sendMissionCommand('cancel_waypoints');
        clearWaypointPreview();
    });
    
    // Step 3: Start/Stop/Resume/Reset Mission
    document.getElementById('btn-start-mission').addEventListener('click', () => {
        sendMissionCommand('start_mission');
    });
    
    document.getElementById('btn-stop-mission').addEventListener('click', () => {
        sendMissionCommand('stop_mission');
    });
    
    document.getElementById('btn-resume-mission').addEventListener('click', () => {
        sendMissionCommand('resume_mission');
    });
    
    document.getElementById('btn-reset-mission').addEventListener('click', () => {
        if (confirm('Сбросить миссию? | Reset mission and clear waypoints?')) {
            sendMissionCommand('reset_mission');
            clearWaypointPreview();
            addLog('Mission reset - ready for new waypoints', 'warning');
        }
    });
    
    // Joystick Override
    document.getElementById('btn-joystick-enable').addEventListener('click', () => {
        sendMissionCommand('joystick_enable');
    });
    
    document.getElementById('btn-joystick-disable').addEventListener('click', () => {
        sendMissionCommand('joystick_disable');
    });
    
    console.log('Mission control initialized');
}

// Generate waypoints and update config
function generateWaypoints() {
    if (!connected || !configPublisher) {
        addLog('Not connected to ROS', 'error');
        return;
    }
    
    // Get waypoint parameters from mission control inputs
    const lanes = parseInt(document.getElementById('wp-lanes').value);
    const length = parseFloat(document.getElementById('wp-length').value);
    const width = parseFloat(document.getElementById('wp-width').value);
    
    // Update hidden config fields for compatibility
    document.getElementById('cfg-lanes').value = lanes;
    document.getElementById('cfg-scan-length').value = length;
    document.getElementById('cfg-scan-width').value = width;
    
    // Don't clear dirty state immediately - wait for ROS confirmation
    
    // Send config update first
    const config = {
        lanes: lanes,
        scan_length: length,
        scan_width: width
    };
    
    const configMsg = new ROSLIB.Message({
        data: JSON.stringify(config)
    });
    configPublisher.publish(configMsg);
    
    // Then send generate command
    sendMissionCommand('generate_waypoints');
    
    // Calculate and display preview info
    const totalWaypoints = lanes * 2 - 1;
    const estimatedDistance = length * lanes + width * (lanes - 1);
    document.getElementById('waypoint-count').textContent = `Waypoints: ~${totalWaypoints}`;
    document.getElementById('estimated-distance').textContent = `Расстояние | Distance: ~${estimatedDistance}m`;
    
    addLog(`Generating ${totalWaypoints} waypoints: ${length}m × ${lanes} lanes`, 'info');
}

// Send mission command to Vostok1
function sendMissionCommand(command) {
    if (!connected) {
        addLog('Not connected to ROS', 'error');
        return;
    }
    
    // Create publisher if not exists
    if (!missionCommandPublisher) {
        missionCommandPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/vostok1/mission_command',
            messageType: 'std_msgs/String'
        });
    }
    
    const msg = new ROSLIB.Message({
        data: JSON.stringify({ command: command })
    });
    
    missionCommandPublisher.publish(msg);
    addLog(`Mission command: ${command}`, 'info');
    console.log('Mission command sent:', command);
}

// Update mission control UI based on state
function updateMissionControlUI(state) {
    missionState.state = state.state || 'IDLE';
    missionState.gpsReady = state.gps_ready || false;
    missionState.missionArmed = state.mission_armed || false;
    missionState.joystickOverride = state.joystick_override || false;
    missionState.totalWaypoints = state.total_waypoints || 0;
    missionState.currentWaypoint = state.current_waypoint || 0;
    missionState.startLat = state.start_lat;
    missionState.startLon = state.start_lon;
    
    // Update GPS indicator
    const gpsBadge = document.getElementById('gps-ready-badge');
    if (gpsBadge) {
        if (missionState.gpsReady) {
            gpsBadge.textContent = '📡 GPS: Готов | Ready';
            gpsBadge.className = 'gps-badge ready';
        } else {
            gpsBadge.textContent = '📡 GPS: Ожидание... | Waiting...';
            gpsBadge.className = 'gps-badge not-ready';
        }
    }
    
    // Update state badge
    const stateBadge = document.getElementById('mission-state-badge');
    if (stateBadge) {
        const stateLabels = {
            'IDLE': 'ОЖИДАНИЕ | IDLE',
            'WAYPOINTS_PREVIEW': '👁️ ПРЕДПРОСМОТР | PREVIEW',
            'READY': '✅ ГОТОВ | READY',
            'RUNNING': '🚀 ВЫПОЛНЯЕТСЯ | RUNNING',
            'PAUSED': '⏸️ ПАУЗА | PAUSED',
            'JOYSTICK': '🎮 ДЖОЙСТИК | JOYSTICK',
            'FINISHED': '🏁 ЗАВЕРШЕНО | FINISHED'
        };
        stateBadge.textContent = stateLabels[missionState.state] || missionState.state;
        stateBadge.className = `mission-badge ${missionState.state.toLowerCase()}`;
    }
    
    // Update button states based on mission state
    const btnGenerate = document.getElementById('btn-generate-waypoints');
    const btnConfirm = document.getElementById('btn-confirm-waypoints');
    const btnCancel = document.getElementById('btn-cancel-waypoints');
    const btnStart = document.getElementById('btn-start-mission');
    const btnStop = document.getElementById('btn-stop-mission');
    const btnResume = document.getElementById('btn-resume-mission');
    const btnJoyEnable = document.getElementById('btn-joystick-enable');
    const btnJoyDisable = document.getElementById('btn-joystick-disable');
    
    // Reset all buttons
    [btnGenerate, btnConfirm, btnCancel, btnStart, btnStop, btnResume, btnJoyEnable, btnJoyDisable].forEach(btn => {
        if (btn) btn.disabled = true;
    });
    
    // Enable buttons based on state
    switch (missionState.state) {
        case 'IDLE':
            if (missionState.gpsReady) {
                btnGenerate.disabled = false;
            }
            btnJoyEnable.disabled = false;
            break;
            
        case 'WAYPOINTS_PREVIEW':
            btnGenerate.disabled = false;
            btnConfirm.disabled = false;
            btnCancel.disabled = false;
            btnJoyEnable.disabled = false;
            break;
            
        case 'READY':
            btnStart.disabled = false;
            btnCancel.disabled = false;
            btnJoyEnable.disabled = false;
            break;
            
        case 'RUNNING':
            btnStop.disabled = false;
            break;
            
        case 'PAUSED':
            btnResume.disabled = false;
            btnStart.disabled = false;
            btnCancel.disabled = false;
            btnJoyEnable.disabled = false;
            break;
            
        case 'JOYSTICK':
            btnJoyDisable.disabled = false;
            break;
            
        case 'FINISHED':
            btnGenerate.disabled = false;
            btnJoyEnable.disabled = false;
            break;
    }
    
    // Update joystick status and instructions display
    const joystickStatus = document.getElementById('joystick-status');
    const joystickInstructions = document.getElementById('joystick-instructions');
    if (joystickStatus) {
        if (missionState.joystickOverride) {
            joystickStatus.classList.remove('hidden');
            if (joystickInstructions) joystickInstructions.classList.remove('hidden');
        } else {
            joystickStatus.classList.add('hidden');
            if (joystickInstructions) joystickInstructions.classList.add('hidden');
        }
    }
    
    // Update waypoint count display
    if (missionState.totalWaypoints > 0) {
        document.getElementById('waypoint-count').textContent = 
            `Waypoints: ${missionState.currentWaypoint}/${missionState.totalWaypoints}`;
    }
}

// Display waypoints on map
// fitToWaypoints: only zoom to fit when waypoints change, not on every update
function displayWaypointsOnMap(waypoints, fitToWaypoints = false) {
    // Clear existing waypoint markers
    clearWaypointPreview();
    
    if (!waypoints || waypoints.length === 0) return;
    if (!missionState.startLat || !missionState.startLon) return;
    
    const startLat = missionState.startLat;
    const startLon = missionState.startLon;
    
    // Convert local coordinates to GPS
    const waypointLatLngs = waypoints.map((wp, idx) => {
        const latLng = localToGPS(wp.x, wp.y, startLat, startLon);
        return { lat: latLng[0], lon: latLng[1], x: wp.x, y: wp.y, idx: idx };
    });
    
    // Create waypoint markers
    waypointLatLngs.forEach((wp, idx) => {
        const isCurrentTarget = idx === missionState.currentWaypoint;
        const isPassed = idx < missionState.currentWaypoint;
        
        const markerColor = isPassed ? 'green' : (isCurrentTarget ? 'orange' : 'blue');
        const markerSize = isCurrentTarget ? 14 : 10;
        const statusText = isPassed ? '✓ Passed' : (isCurrentTarget ? '→ Current Target' : 'Pending');
        
        const icon = L.divIcon({
            className: 'waypoint-marker',
            html: `<div style="
                background-color: ${markerColor};
                width: ${markerSize}px;
                height: ${markerSize}px;
                border-radius: 50%;
                border: 2px solid white;
                box-shadow: 0 0 4px rgba(0,0,0,0.5);
                cursor: pointer;
            "></div>`,
            iconSize: [markerSize, markerSize],
            iconAnchor: [markerSize/2, markerSize/2]
        });
        
        // Detailed tooltip content
        const tooltipContent = `
            <div class="waypoint-tooltip">
                <strong>Waypoint ${idx + 1}/${waypoints.length}</strong><br>
                <span class="wp-status" style="color: ${markerColor}">${statusText}</span>
                <hr style="margin: 4px 0; border-color: #ddd;">
                <b>Local:</b> (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)}) m<br>
                <b>GPS:</b> ${wp.lat.toFixed(6)}°, ${wp.lon.toFixed(6)}°
            </div>
        `;
        
        const marker = L.marker([wp.lat, wp.lon], { icon: icon })
            .bindTooltip(tooltipContent, {
                permanent: false,
                direction: 'top',
                offset: [0, -10],
                className: 'waypoint-tooltip-container',
                sticky: true  // Keeps tooltip visible while hovering
            })
            .addTo(map);
        
        waypointMarkers.push(marker);
    });
    
    // Draw path line connecting waypoints
    const pathCoords = waypointLatLngs.map(wp => [wp.lat, wp.lon]);
    waypointPath = L.polyline(pathCoords, {
        color: '#3388ff',
        weight: 2,
        opacity: 0.7,
        dashArray: '5, 10'
    }).addTo(map);
    
    // Fit map to show all waypoints (only when waypoints change, not on every update)
    if (fitToWaypoints && pathCoords.length > 0) {
        const bounds = L.latLngBounds(pathCoords);
        bounds.extend([startLat, startLon]);  // Include boat position
        map.fitBounds(bounds, { padding: [50, 50] });
    }
    
    addLog(`Displayed ${waypoints.length} waypoints on map`, 'info');
}

// Clear waypoint preview from map
function clearWaypointPreview() {
    waypointMarkers.forEach(marker => map.removeLayer(marker));
    waypointMarkers = [];
    
    if (waypointPath) {
        map.removeLayer(waypointPath);
        waypointPath = null;
    }
}

// Convert local ENU coordinates to GPS
function localToGPS(x, y, refLat, refLon) {
    const R = 6371000.0;  // Earth radius in meters
    const refLatRad = refLat * Math.PI / 180;
    
    // x = East, y = North
    const dLat = y / R;
    const dLon = x / (R * Math.cos(refLatRad));
    
    const lat = refLat + dLat * 180 / Math.PI;
    const lon = refLon + dLon * 180 / Math.PI;
    
    return [lat, lon];
}
