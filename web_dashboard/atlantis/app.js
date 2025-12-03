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
let rosoutSub = null;

// Publishers
let replanPub = null;
let startPub = null;
let stopPub = null;

// State
let boatPosition = { lat: 0, lon: 0 };
let waypoints = [];
let currentWpIndex = 0;
let startPosition = { lat: null, lon: null };

// Map
let map = null;
let boatMarker = null;
let pathLine = null;
let waypointMarkers = [];
let currentWaypointMarker = null;
let mapFollowBoat = true;  // Default to following boat
let trajectoryLine = null;
let trajectoryPoints = [];

// ============================================
// INITIALIZATION
// ============================================
document.addEventListener('DOMContentLoaded', () => {
    initMap();
    initUI();
    initTerminal();
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

    // Add Follow Boat Control
    const FollowBoatControl = L.Control.extend({
        options: { position: 'topright' },
        onAdd: function(map) {
            const container = L.DomUtil.create('div', 'follow-boat-control leaflet-bar');
            const button = L.DomUtil.create('button', 'follow-boat-btn active', container);
            button.innerHTML = '🎯';
            button.title = 'Follow boat (click to toggle)';
            
            L.DomEvent.on(button, 'click', function(e) {
                L.DomEvent.stopPropagation(e);
                mapFollowBoat = !mapFollowBoat;
                if (mapFollowBoat) {
                    button.classList.add('active');
                    button.innerHTML = '🎯';
                    button.title = 'Following boat (click to unlock)';
                    // Center on boat immediately
                    if (boatPosition.lat && boatPosition.lon) {
                        map.panTo([boatPosition.lat, boatPosition.lon]);
                    }
                } else {
                    button.classList.remove('active');
                    button.innerHTML = '🔓';
                    button.title = 'Free movement (click to follow boat)';
                }
            });
            
            return container;
        }
    });
    map.addControl(new FollowBoatControl());

    // Boat marker with custom icon
    const boatIcon = L.divIcon({
        className: 'boat-marker',
        html: '<div style="font-size: 24px;">🚤</div>',
        iconSize: [32, 32],
        iconAnchor: [16, 16]
    });
    boatMarker = L.marker([-33.724, 151.0], { icon: boatIcon }).addTo(map);

    // Planned path line
    pathLine = L.polyline([], {
        color: '#667eea',
        weight: 3,
        opacity: 0.8,
        dashArray: '10, 5'
    }).addTo(map);

    // Trajectory line (actual path traveled)
    trajectoryLine = L.polyline([], {
        color: '#22c55e',
        weight: 2,
        opacity: 0.6
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
    document.getElementById('btn-clear-terminal').addEventListener('click', clearTerminal);
    
    // Theme toggle initialization
    initThemeToggle();
}

// ============================================
// THEME SWITCHER - 1980s Soviet/TNO Style
// ============================================
function initThemeToggle() {
    const themeBtn = document.getElementById('theme-toggle-btn');
    const themeIcon = themeBtn.querySelector('.theme-icon');
    const themeText = themeBtn.querySelector('.theme-text');
    
    // Load saved theme preference
    const savedTheme = localStorage.getItem('atlantis-theme');
    if (savedTheme === 'soviet') {
        document.body.classList.add('theme-soviet');
        themeIcon.textContent = '🌐';
        themeText.textContent = 'MODERN';
    }
    
    themeBtn.addEventListener('click', () => {
        const isSoviet = document.body.classList.toggle('theme-soviet');
        
        if (isSoviet) {
            // Switched to Soviet theme
            themeIcon.textContent = '🌐';
            themeText.textContent = 'MODERN';
            localStorage.setItem('atlantis-theme', 'soviet');
            log('СИСТЕМА АКТИВИРОВАНА — 1980 USSR MODE', 'info');
            terminalLog('>>> ПЕРЕКЛЮЧЕНИЕ НА СОВЕТСКИЙ РЕЖИМ <<<', 'system');
            terminalLog('СИСТЕМА КОНТРОЛЯ СУДНА АКТИВНА', 'system');
        } else {
            // Switched to Modern theme
            themeIcon.textContent = '☭';
            themeText.textContent = 'USSR 1980';
            localStorage.setItem('atlantis-theme', 'modern');
            log('Switched to Modern theme', 'info');
            terminalLog('Theme switched to Modern', 'system');
        }
    });
}

function initTerminal() {
    // Terminal is already initialized via HTML
    terminalLog('Terminal initialized. Connect to ROS to see node output.', 'system');
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

    // Subscribe to ROS logs (rosout) for terminal
    rosoutSub = new ROSLIB.Topic({
        ros: ros,
        name: '/rosout',
        messageType: 'rcl_interfaces/Log'
    });
    rosoutSub.subscribe((msg) => {
        // Filter for atlantis node messages
        if (msg.name && (
            msg.name.includes('atlantis') ||
            msg.name.includes('planner') ||
            msg.name.includes('controller')
        )) {
            handleRosLog(msg);
        }
    });

    terminalLog('Connected to ROS - receiving node output', 'system');
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
        
        // Update current waypoint index for map display
        const newWpIndex = data.waypoint - 1;  // Convert to 0-based index
        if (newWpIndex !== currentWpIndex && waypoints.length > 0) {
            currentWpIndex = newWpIndex;
            displayWaypointsOnMap(waypoints);  // Refresh waypoint colors
        }
        
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
    
    // Store first GPS as start position
    if (startPosition.lat === null) {
        startPosition.lat = msg.latitude;
        startPosition.lon = msg.longitude;
    }
    
    document.getElementById('gps-status').textContent = `${msg.latitude.toFixed(6)}, ${msg.longitude.toFixed(6)}`;
    
    // Update boat marker
    boatMarker.setLatLng([msg.latitude, msg.longitude]);
    
    // Add to trajectory
    trajectoryPoints.push([msg.latitude, msg.longitude]);
    if (trajectoryPoints.length > 200) {
        trajectoryPoints.shift();  // Keep last 200 points
    }
    trajectoryLine.setLatLngs(trajectoryPoints);
    
    // Center map on boat only if follow mode is enabled
    if (mapFollowBoat) {
        const center = map.getCenter();
        const distance = map.distance(center, [msg.latitude, msg.longitude]);
        if (distance > 30) {  // More than 30m from center
            map.panTo([msg.latitude, msg.longitude]);
        }
    }
}

function handlePath(msg) {
    // Clear existing waypoint markers
    clearWaypointMarkers();
    
    waypoints = msg.poses.map(p => ({
        x: p.pose.position.x,
        y: p.pose.position.y
    }));
    
    log(`Received path with ${waypoints.length} waypoints`, 'info');
    terminalLog(`Path received: ${waypoints.length} waypoints`, 'info');
    
    // Update waypoint info display
    document.getElementById('waypoint-info').textContent = `${waypoints.length} waypoints`;
    
    // Display waypoints on map if we have GPS reference
    if (startPosition.lat !== null && waypoints.length > 0) {
        displayWaypointsOnMap(waypoints);
    }
}

function displayWaypointsOnMap(waypoints) {
    // Clear existing markers
    clearWaypointMarkers();
    
    if (!waypoints || waypoints.length === 0) return;
    if (!startPosition.lat || !startPosition.lon) return;
    
    const pathCoords = [];
    
    waypoints.forEach((wp, idx) => {
        // Convert local ENU to GPS
        const gps = localToGPS(wp.x, wp.y, startPosition.lat, startPosition.lon);
        pathCoords.push(gps);
        
        // Determine marker style based on waypoint status
        const isCurrentTarget = idx === currentWpIndex;
        const isPassed = idx < currentWpIndex;
        
        const markerColor = isPassed ? '#22c55e' : (isCurrentTarget ? '#f59e0b' : '#764ba2');
        const markerSize = isCurrentTarget ? 14 : 10;
        const statusText = isPassed ? '✓ Passed' : (isCurrentTarget ? '→ Current' : 'Pending');
        
        const icon = L.divIcon({
            className: 'waypoint-marker',
            html: `<div style="
                background-color: ${markerColor};
                width: ${markerSize}px;
                height: ${markerSize}px;
                border-radius: 50%;
                border: 2px solid white;
                box-shadow: 0 0 4px rgba(0,0,0,0.5);
            "></div>`,
            iconSize: [markerSize, markerSize],
            iconAnchor: [markerSize/2, markerSize/2]
        });
        
        const tooltipContent = `
            <div class="waypoint-tooltip">
                <strong>Waypoint ${idx + 1}/${waypoints.length}</strong><br>
                <span style="color: ${markerColor}">${statusText}</span>
                <hr>
                <b>Local:</b> (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)}) m<br>
                <b>GPS:</b> ${gps[0].toFixed(6)}°, ${gps[1].toFixed(6)}°
            </div>
        `;
        
        const marker = L.marker(gps, { icon: icon })
            .bindTooltip(tooltipContent, {
                permanent: false,
                direction: 'top',
                offset: [0, -10]
            })
            .addTo(map);
        
        waypointMarkers.push(marker);
    });
    
    // Update path line
    pathLine.setLatLngs(pathCoords);
    
    // Fit map to show all waypoints (only first time)
    if (pathCoords.length > 0 && !mapFollowBoat) {
        const bounds = L.latLngBounds(pathCoords);
        if (boatPosition.lat) {
            bounds.extend([boatPosition.lat, boatPosition.lon]);
        }
        map.fitBounds(bounds, { padding: [50, 50] });
    }
}

function clearWaypointMarkers() {
    waypointMarkers.forEach(marker => map.removeLayer(marker));
    waypointMarkers = [];
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

// ============================================
// TERMINAL OUTPUT
// ============================================
function handleRosLog(msg) {
    // ROS log levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
    let level = 'info';
    let levelText = 'INFO';
    
    if (msg.level >= 40) {
        level = 'error';
        levelText = 'ERROR';
    } else if (msg.level >= 30) {
        level = 'warn';
        levelText = 'WARN';
    } else if (msg.level <= 10) {
        level = 'debug';
        levelText = 'DEBUG';
    }
    
    const terminal = document.getElementById('terminal-output');
    const line = document.createElement('div');
    line.className = `terminal-line ${level}`;
    
    const now = new Date();
    const timestamp = now.toLocaleTimeString('en-US', { hour12: false });
    
    line.innerHTML = `<span class="timestamp">[${timestamp}]</span> <span class="level">[${levelText}]</span> <span class="message">${msg.msg}</span>`;
    terminal.appendChild(line);
    
    // Auto-scroll if enabled
    const autoScroll = document.getElementById('terminal-auto-scroll');
    if (autoScroll && autoScroll.checked) {
        terminal.scrollTop = terminal.scrollHeight;
    }
    
    // Limit lines
    while (terminal.children.length > 200) {
        terminal.removeChild(terminal.firstChild);
    }
}

function terminalLog(message, level = 'info') {
    const terminal = document.getElementById('terminal-output');
    const line = document.createElement('div');
    line.className = `terminal-line ${level}`;
    
    const now = new Date();
    const timestamp = now.toLocaleTimeString('en-US', { hour12: false });
    const levelText = level.toUpperCase();
    
    line.innerHTML = `<span class="timestamp">[${timestamp}]</span> <span class="level">[${levelText}]</span> <span class="message">${message}</span>`;
    terminal.appendChild(line);
    
    // Auto-scroll if enabled
    const autoScroll = document.getElementById('terminal-auto-scroll');
    if (autoScroll && autoScroll.checked) {
        terminal.scrollTop = terminal.scrollHeight;
    }
}

function clearTerminal() {
    const terminal = document.getElementById('terminal-output');
    terminal.innerHTML = '<div class="terminal-line system"><span class="level">[SYSTEM]</span> <span class="message">Terminal cleared.</span></div>';
}
