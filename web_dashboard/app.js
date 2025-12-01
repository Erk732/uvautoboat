// ROS Connection
let ros;
let connected = false;

// Map variables
let map;
let boatMarker;
let trajectoryLine;
let trajectoryPoints = [];
let waypointMarkers = [];

// Configuration publisher
let configPublisher = null;

// Data storage
let currentState = {
    gps: { lat: 0, lon: 0, local_x: 0, local_y: 0 },
    obstacles: { min: Infinity, front: true, left: true, right: true },
    thrusters: { left: 0, right: 0 },
    mission: { state: 'INIT', waypoint: 0, distance: 0 },
    config: {
        scan_length: 15.0,
        scan_width: 30.0,
        lanes: 10,
        kp: 400.0,
        ki: 20.0,
        kd: 100.0,
        base_speed: 500.0,
        max_speed: 800.0,
        min_safe_distance: 15.0
    }
};

// Style mode state
let isTerminalMode = false;

// Initialize everything when page loads
window.addEventListener('load', () => {
    console.log('Dashboard loading...');
    initMap();
    connectToROS();
    initStyleToggle();
    initConfigPanel();
    initTerminal();
    addLog('Dashboard initialized', 'info');
});

// Initialize style toggle button
function initStyleToggle() {
    const toggleBtn = document.getElementById('style-toggle');
    const toggleText = document.getElementById('toggle-text');
    const body = document.body;
    const container = document.querySelector('.container');
    
    toggleBtn.addEventListener('click', () => {
        isTerminalMode = !isTerminalMode;
        
        if (isTerminalMode) {
            body.classList.add('terminal-mode');
            container.classList.add('terminal-mode');
            toggleText.textContent = 'БЮРО | BUREAU MODE';
            addLog('Переключено на режим терминала | Switched to terminal mode', 'info');
        } else {
            body.classList.remove('terminal-mode');
            container.classList.remove('terminal-mode');
            toggleText.textContent = 'ТЕРМИНАЛ | TERMINAL MODE';
            addLog('Переключено на режим бюро | Switched to bureau mode', 'info');
        }
    });
    
    console.log('Style toggle initialized');
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
        updateMissionStatus({
            state: data.state,
            waypoint: data.current_waypoint,
            total_waypoints: data.total_waypoints,
            distance_to_waypoint: 0  // Not provided in modular format
        });
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
    
    // Create publisher for configuration updates
    configPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/set_config',
        messageType: 'std_msgs/String'
    });
    
    // Subscribe to current config
    const configTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/config',
        messageType: 'std_msgs/String'
    });
    
    configTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        console.log('Config received:', data);
        updateConfigFromROS(data);
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
    
    // Center map on boat (only if far from current view)
    const center = map.getCenter();
    const distance = map.distance(center, latLng);
    if (distance > 100) { // More than 100m from center
        map.panTo(latLng);
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
    
    // Apply all config button
    document.getElementById('btn-apply-config').addEventListener('click', () => {
        sendConfig(false, false);
    });
    
    // Apply PID only button
    document.getElementById('btn-apply-pid').addEventListener('click', () => {
        sendConfig(true, false);
    });
    
    // Restart mission button
    document.getElementById('btn-restart-mission').addEventListener('click', () => {
        sendConfig(false, true);
    });
    
    console.log('Config panel initialized');
}

// Update config inputs from ROS
function updateConfigFromROS(data) {
    currentState.config = { ...currentState.config, ...data };
    
    // Only update inputs if they're not focused (user not typing)
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
    
    for (const [id, value] of Object.entries(inputs)) {
        const el = document.getElementById(id);
        if (el && document.activeElement !== el && value !== undefined) {
            el.value = value;
        }
    }
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
    
    configPublisher.publish(message);
    addLog('Конфигурация отправлена | Config sent!', 'info');
    console.log('Config sent:', config);
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
