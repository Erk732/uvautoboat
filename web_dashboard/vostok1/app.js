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
let modularMissionCommandPublisher = null;  // For sputnik planner

// Track which config inputs have been modified by user (prevents ROS from overwriting)
let dirtyInputs = new Set();
let navModeDirty = false;
let pollutantMarkers = [];

// Camera feed elements
let cameraImageEl = null;
let cameraStatusEl = null;
let cameraTopicInput = null;

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

const WAYPOINT_STORAGE_KEY = 'vostok1_cached_waypoints';

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

// Style mode (single mode - normal only)

// Initialize everything when page loads
window.addEventListener('load', () => {
    console.log('Dashboard loading...');
    initMap();
    restorePersistedWaypoints();  // Recover waypoints after hard refresh
    connectToROS();
    initStyleToggle();
    initConfigPanel();
    initMissionControl();  // NEW: Mission control buttons
    initCameraFeed();      // Camera/RViz stream panel
    initTerminal();
    addLog('Dashboard initialized', 'info');
});

// Style toggle removed - using single normal mode
function initStyleToggle() {
    // Style toggle functionality removed
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
            btn.title = 'Following active (click to disable) | Suivi actif';
        } else {
            btn.innerHTML = '🎯';
            btn.classList.remove('active');
            btn.title = 'Click to follow boat | Suivre le navire';
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
            button.title = 'Follow boat | Suivre le navire';
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

// Initialize camera/RViz stream panel using web_video_server
function initCameraFeed() {
    cameraImageEl = document.getElementById('camera-image');
    cameraStatusEl = document.getElementById('camera-status');
    cameraTopicInput = document.getElementById('camera-topic');
    const refreshBtn = document.getElementById('btn-refresh-camera');

    if (refreshBtn) {
        refreshBtn.addEventListener('click', updateCameraStream);
    }

    if (!cameraImageEl || !cameraStatusEl || !cameraTopicInput) {
        return;
    }

    cameraImageEl.addEventListener('load', () => {
        cameraStatusEl.textContent = 'Streaming | Flux en cours';
        cameraStatusEl.classList.remove('error');
    });

    cameraImageEl.addEventListener('error', () => {
        cameraStatusEl.textContent = 'Camera feed unavailable. Start web_video_server? | Flux indisponible';
        cameraStatusEl.classList.add('error');
    });

    updateCameraStream();
}

function updateCameraStream() {
    if (!cameraImageEl || !cameraTopicInput || !cameraStatusEl) {
        return;
    }
    const topic = cameraTopicInput.value.trim() || '/wamv/sensors/cameras/front_left_camera_sensor/image_raw';
    cameraStatusEl.textContent = `Connecting to ${topic}...`;
    const streamUrl = buildCameraUrl(topic);
    // Cache-bust each refresh to force a reconnect
    cameraImageEl.src = `${streamUrl}&t=${Date.now()}`;
}

function buildCameraUrl(topic) {
    // Use the current host so you don't need a separate tab on another localhost/host.
    const baseHost = window.location.hostname || 'localhost';
    const base = `${window.location.protocol}//${baseHost}:8080/stream`;
    // web_video_server expects the topic path unencoded (slashes are valid), so build manually.
    return `${base}?topic=${topic}&type=mjpeg&quality=80`;
}

// Update connection status indicator
function updateConnectionStatus(isConnected) {
    const statusElement = document.getElementById('connection-status');
    if (isConnected) {
        statusElement.textContent = 'Connected | Connecté';
        statusElement.className = 'status connected';
    } else {
        statusElement.textContent = 'Disconnected | Déconnecté';
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
            persistWaypoints();
            displayWaypointsOnMap(data.waypoints, waypointsChanged);
        } else {
            clearWaypoints('Planner cleared waypoints');
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
        // Convert modular format to vostok1 format with OKO v2.0/v2.1 enhancements
        // OKO v2.0: front_clear, left_clear, right_clear ARE the distances in meters
        const CLEAR_THRESHOLD = 10.0;  // Distance threshold for "clear" status
        updateObstacleStatus({
            min_distance: data.min_distance,
            // OKO v2.0: Use distance values for both boolean and numeric display
            front_clear: data.front_clear > CLEAR_THRESHOLD,
            left_clear: data.left_clear > CLEAR_THRESHOLD,
            right_clear: data.right_clear > CLEAR_THRESHOLD,
            front_distance: data.front_clear,  // OKO v2.0: front_clear IS the distance
            left_distance: data.left_clear,    // OKO v2.0: left_clear IS the distance
            right_distance: data.right_clear,  // OKO v2.0: right_clear IS the distance
            status: data.force_avoid_active ? '🔴 FORCE AVOID | Évitement forcé' :
                    data.is_critical ? '🚨 CRITICAL | Critique' :
                    data.obstacle_detected ? '⚠️ OBSTACLE | Détecté' :
                    '✅ CLEAR | Dégagé',
            // OKO v2.0 enhanced fields
            urgency: data.urgency || 0.0,
            obstacle_count: data.obstacle_count || 0,
            best_gap: data.best_gap || null,
            clusters: data.clusters || [],
            moving_obstacles: data.moving_obstacles || [],  // OKO v2.0: array of {id, vx, vy, speed}
            // OKO v2.1 enhanced fields
            vfh_gap: data.vfh_gap || null,           // VFH best direction
            polar_bias: data.polar_bias || 0.0,      // Steering bias [-1, 1]
            force_avoid_active: data.force_avoid_active || false,
            laserscan_fused: data.laserscan_fused || false
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

    // Pollutant sources (smoke generators) for minimap markers
    const pollutantTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/perception/pollutant_sources',
        messageType: 'std_msgs/String'
    });

    pollutantTopic.subscribe((message) => {
        const data = JSON.parse(message.data);
        if (data.sources) {
            updatePollutantSources(data.sources);
        }
    });

    console.log('Subscribed to all topics (integrated + modular)');
    addLog('Subscribed to topics (Vostok1 + Modular)', 'info');
    
    // Enable mission control UI when connected (before config received)
    updateMissionControlUI({ state: 'IDLE', gps_ready: true });
    
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
            persistWaypoints();
            displayWaypointsOnMap(data.waypoints, waypointsChanged);
        } else {
            clearWaypoints('Waypoints cleared');
        }
    });
    
    // Create mission command publisher (supports both modes)
    missionCommandPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/vostok1/mission_command',
        messageType: 'std_msgs/String'
    });
    
    // Create modular mission command publisher for sputnik
    modularMissionCommandPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/sputnik/mission_command',
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
    
    addTerminalLine({ level: 20, msg: 'Connected to ROS | Connecté à ROS', name: 'system' });
}

// ========== CAMERA STREAMING ==========
let cameraTopic = null;
let cameraImage = document.getElementById('camera-image');
let cameraStatus = document.getElementById('camera-status');

function initializeCamera() {
    if (!connected) {
        console.log('ROS not connected, skipping camera initialization');
        return;
    }
    
    const defaultTopic = '/wamv/sensors/camera/image_raw';
    subscribeToCamera(defaultTopic);
    
    // Set up refresh button
    document.getElementById('btn-refresh-camera').addEventListener('click', () => {
        const topic = document.getElementById('camera-topic').value;
        if (cameraTopic) {
            cameraTopic.unsubscribe();
        }
        subscribeToCamera(topic);
    });
}

function subscribeToCamera(topicName) {
    try {
        // Create video stream using ros bridge
        cameraTopic = new ROSLIB.Topic({
            ros: ros,
            name: topicName,
            messageType: 'sensor_msgs/Image'
        });
        
        // Subscribe and display images
        cameraTopic.subscribe((message) => {
            try {
                // Convert ROS image message to base64 and display
                const imageData = message.data;
                if (imageData && imageData.length > 0) {
                    // Decode base64 image
                    const binaryString = atob(imageData);
                    const bytes = new Uint8Array(binaryString.length);
                    for (let i = 0; i < binaryString.length; i++) {
                        bytes[i] = binaryString.charCodeAt(i);
                    }
                    const blob = new Blob([bytes], { type: 'image/jpeg' });
                    const url = URL.createObjectURL(blob);
                    
                    cameraImage.src = url;
                    cameraStatus.style.display = 'none';
                    cameraImage.style.display = 'block';
                }
            } catch (e) {
                console.error('Error processing camera image:', e);
                cameraStatus.textContent = `Error: ${e.message}`;
                cameraStatus.style.display = 'block';
            }
        });
        
        console.log(`✓ Camera subscribed to: ${topicName}`);
        cameraStatus.textContent = `Camera: ${topicName}`;
        cameraStatus.style.display = 'block';
        addLog(`✓ Camera feed initialized | Flux caméra initialisé: ${topicName}`, 'info');
        
    } catch (error) {
        console.error('Error subscribing to camera topic:', error);
        cameraStatus.textContent = `Error: Cannot connect to ${topicName}`;
        cameraStatus.style.display = 'block';
        addLog(`✗ Camera initialization failed | Échec: ${error.message}`, 'error');
    }
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
    if (trajectoryPoints.length > 300) {
        trajectoryPoints.shift(); // Keep last 300 points (extended trail)
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
    currentState.mission.state = data.state || 'UNKNOWN';
    currentState.mission.waypoint = data.waypoint || 0;
    // Guard against missing distance fields to avoid crashes on refresh
    const distance = (data.distance_to_waypoint !== undefined && data.distance_to_waypoint !== null)
        ? data.distance_to_waypoint
        : 0;
    currentState.mission.distance = distance;
    
    // CRITICAL: Update missionState.currentWaypoint so waypoints on map reflect current progress
    // This is needed for displayWaypointsOnMap() to show correct waypoint coloring (passed/current/pending)
    const previousWaypoint = missionState.currentWaypoint;
    missionState.currentWaypoint = data.waypoint || 0;
    // Sync totals if provided
    if (data.total_waypoints !== undefined) {
        missionState.totalWaypoints = data.total_waypoints || 0;
        // If planner reports zero waypoints, clear map so stale routes disappear
        if (missionState.totalWaypoints === 0 && missionState.waypoints.length > 0) {
            clearWaypoints('Planner reported 0 waypoints');
        } else {
            persistWaypoints();
        }
    }
    
    // If waypoint changed, redraw waypoints on map to update colors (green for passed, orange for current)
    if (previousWaypoint !== missionState.currentWaypoint && missionState.waypoints.length > 0) {
        console.log(`Waypoint updated: ${previousWaypoint} → ${missionState.currentWaypoint}, redrawing map`);
        displayWaypointsOnMap(missionState.waypoints, false);  // Don't fit to bounds, just update colors
    }
    
    document.getElementById('state').textContent = (data.state || 'UNKNOWN').replace(/_/g, ' ');
    document.getElementById('waypoint').textContent = `${data.waypoint || 0}/${data.total_waypoints || missionState.totalWaypoints || 0}`;
    document.getElementById('distance').textContent = distance.toFixed(1) + 'm';
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
        data.front_clear ? `✓ ${data.front_distance.toFixed(1)}m` : `✗ ${data.front_distance.toFixed(1)}m`;
    document.getElementById('left-clear').textContent = 
        data.left_clear ? `✓ ${data.left_distance.toFixed(1)}m` : `✗ ${data.left_distance.toFixed(1)}m`;
    document.getElementById('right-clear').textContent = 
        data.right_clear ? `✓ ${data.right_distance.toFixed(1)}m` : `✗ ${data.right_distance.toFixed(1)}m`;
    
    // OKO v2.0: Update urgency display if element exists
    const urgencyEl = document.getElementById('urgency');
    if (urgencyEl && data.urgency !== undefined) {
        const urgencyPct = (data.urgency * 100).toFixed(0);
        urgencyEl.textContent = `${urgencyPct}%`;
        urgencyEl.className = data.urgency > 0.7 ? 'value critical' : 
                              data.urgency > 0.3 ? 'value warning' : 'value';
    }
    
    // OKO v2.0: Update obstacle count if element exists
    const countEl = document.getElementById('obstacle-count');
    if (countEl && data.obstacle_count !== undefined) {
        countEl.textContent = data.obstacle_count;
    }
    
    // OKO v2.0: Update best gap if element exists
    const gapEl = document.getElementById('best-gap');
    if (gapEl && data.best_gap) {
        const gap = data.best_gap;
        gapEl.textContent = `${gap.direction.toFixed(0)}° (${gap.width.toFixed(0)}°)`;
    }
    
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
            statusBadge.textContent = 'Clear | Dégagé';
            statusBadge.className = 'value badge clear';
        } else if (minDist > 5) {
            statusBadge.textContent = 'Warning | Attention';
            statusBadge.className = 'value badge warning';
        } else {
            statusBadge.textContent = 'Critical | Critique';
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
            stuckStatus.textContent = `STUCK | BLOQUÉ (Attempt ${data.consecutive_attempts})`;
            stuckStatus.className = 'value badge critical';
        } else {
            stuckStatus.textContent = 'Normal';
            stuckStatus.className = 'value badge clear';
        }
    }

    if (escapePhase) {
        const phases = ['PROBE', 'REVERSE', 'TURN', 'FORWARD', 'IDLE'];
        const phaseNames = ['PROBE | Sondage', 'REVERSE | Arrière', 'TURN | Virage', 'FORWARD | Avant', 'IDLE | Attente'];
        const idx = data.escape_mode ? data.escape_phase : 4;
        escapePhase.textContent = phaseNames[idx];
        escapePhase.className = data.escape_mode ? 'value active' : 'value';
    }
    
    if (noGoZones) {
        noGoZones.textContent = `${data.no_go_zones}`;
    }
    
    if (driftVector) {
        const dx = data.drift_vector[0];
        const dy = data.drift_vector[1];
        const magnitude = Math.hypot(dx, dy);
        if (magnitude > 0.1) {
            const angle = Math.atan2(dy, dx) * 180 / Math.PI;
            driftVector.textContent = `${magnitude.toFixed(2)} m/s @ ${angle.toFixed(0)}°`;
        } else {
            driftVector.textContent = 'Minimal';
        }
    }
    
    // Display Kalman filter uncertainty
    const driftUncertainty = document.getElementById('drift-uncertainty');
    if (driftUncertainty && data.drift_uncertainty) {
        const ux = data.drift_uncertainty[0];
        const uy = data.drift_uncertainty[1];
        const avgUncertainty = Math.hypot(ux, uy);
        if (avgUncertainty < 0.1) {
            driftUncertainty.textContent = 'High conf. | Haute conf.';
            driftUncertainty.style.color = '#4CAF50';
        } else if (avgUncertainty < 0.5) {
            driftUncertainty.textContent = `σ=${avgUncertainty.toFixed(2)}`;
            driftUncertainty.style.color = '#FFC107';
        } else {
            driftUncertainty.textContent = `σ=${avgUncertainty.toFixed(2)} (converging | convergence)`;
            driftUncertainty.style.color = '#FF9800';
        }
    }

    if (escapeHistory) {
        escapeHistory.textContent = `${data.escape_history_count} entries | entrées`;
    }

    if (probeResults && data.probe_results) {
        probeResults.textContent = `L:${data.probe_results.left}m | R:${data.probe_results.right}m`;
    }

    // Update best direction indicator if exists
    const bestDirection = document.getElementById('best-direction');
    if (bestDirection && data.best_direction) {
        bestDirection.textContent = data.best_direction === 'LEFT' ? '← LEFT | Gauche' : '→ RIGHT | Droite';
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
        'cfg-astar-resolution', 'cfg-astar-safety', 'cfg-astar-max',
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

    // Navigation mode radio buttons - show/hide advanced A* params
    const navModeRadios = document.querySelectorAll('input[name="nav-mode"]');
    const advancedParams = document.getElementById('astar-advanced-params');

    navModeRadios.forEach(radio => {
        radio.addEventListener('change', () => {
            navModeDirty = true;
            // Show advanced params if Runtime A* or Hybrid mode selected
            if (radio.value === 'runtime' || radio.value === 'hybrid') {
                advancedParams.style.display = 'block';
            } else {
                advancedParams.style.display = 'none';
            }
        });
    });

    // Apply all config button (applies all parameters including PID, speed, and scan settings)
    document.getElementById('btn-apply-config').addEventListener('click', () => {
        sendConfig(false, false);  // Send ALL parameters (PID, speed, scan settings)
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

    // Update navigation mode radio buttons based on A* settings (respect user edits)
    if (!navModeDirty && (data.astar_hybrid_mode !== undefined || data.astar_enabled !== undefined)) {
        let navMode = 'simple';
        if (data.astar_hybrid_mode === true) {
            navMode = 'hybrid';
        } else if (data.astar_enabled === true) {
            navMode = 'runtime';
        }
        const rb = document.getElementById(`nav-mode-${navMode}`);
        if (rb) rb.checked = true;
        const advancedParams = document.getElementById('astar-advanced-params');
        if (advancedParams) advancedParams.style.display = (navMode === 'runtime' || navMode === 'hybrid') ? 'block' : 'none';
    } else if (navModeDirty) {
        // If incoming matches current selection, clear dirty
        const current = getSelectedNavMode();
        const incoming = data.astar_hybrid_mode ? 'hybrid' : (data.astar_enabled ? 'runtime' : 'simple');
        if (current === incoming) {
            navModeDirty = false;
        }
    }

    // Update A* advanced parameters if present
    if (data.astar_resolution !== undefined) {
        const el = document.getElementById('cfg-astar-resolution');
        if (el) el.value = data.astar_resolution;
    }
    if (data.astar_safety_margin !== undefined) {
        const el = document.getElementById('cfg-astar-safety');
        if (el) el.value = data.astar_safety_margin;
    }
    if (data.astar_max_expansions !== undefined) {
        const el = document.getElementById('cfg-astar-max');
        if (el) el.value = data.astar_max_expansions;
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
        addLog('Sending PID config... | Envoi configuration PID...', 'info');
    } else {
        // Get selected navigation mode
        const navMode = getSelectedNavMode();

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
            min_safe_distance: parseFloat(document.getElementById('cfg-safe-dist').value),
            // A* detour options based on selected navigation mode
            astar_enabled: (navMode === 'runtime' || navMode === 'hybrid'),
            astar_hybrid_mode: (navMode === 'hybrid'),
            hazard_enabled: (navMode === 'hybrid'),
            astar_resolution: parseFloat(document.getElementById('cfg-astar-resolution').value),
            astar_safety_margin: parseFloat(document.getElementById('cfg-astar-safety').value),
            astar_max_expansions: parseInt(document.getElementById('cfg-astar-max').value)
        };
        addLog('Sending full config... | Envoi configuration complète...', 'info');
    }

    if (restart) {
        config.restart_mission = true;
        addLog('Restarting mission... | Redémarrage de la mission...', 'warning');
    }
    
    const message = new ROSLIB.Message({
        data: JSON.stringify(config)
    });
    
    // Publish to both vostok1 and modular (sputnik) topics
    configPublisher.publish(message);
    if (modularConfigPublisher) {
        modularConfigPublisher.publish(message);
    }
    addLog('Config sent! | Configuration envoyée!', 'info');
    console.log('Config sent to both vostok1 and sputnik:', config);
}

function getSelectedNavMode() {
    const radio = document.querySelector('input[name="nav-mode"]:checked');
    return radio ? radio.value : 'simple';
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
            terminal.innerHTML = '<div class="terminal-line system">[SYSTEM] Terminal cleared | Terminal effacé</div>';
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
        // Send a second stop shortly after to ensure controllers see it (helps during SASS)
        setTimeout(() => sendMissionCommand('stop_mission'), 200);
    });
    
    document.getElementById('btn-resume-mission').addEventListener('click', () => {
        sendMissionCommand('resume_mission');
    });
    
    document.getElementById('btn-reset-mission').addEventListener('click', () => {
        if (confirm('Reset mission and clear waypoints? | Réinitialiser la mission et effacer les waypoints?')) {
            sendMissionCommand('reset_mission');
            clearWaypointPreview();
            addLog('Mission reset - ready for new waypoints | Prêt pour nouveaux waypoints', 'warning');
        }
    });

    // Go Home - One-click return to spawn point
    document.getElementById('btn-go-home').addEventListener('click', () => {
        if (confirm('🏠 Go Home: The boat will navigate to its starting point. Continue? | Le bateau va naviguer vers son point de départ. Continuer?')) {
            sendMissionCommand('go_home');
            addLog('🏠 Go Home activated | Retour maison activé', 'info');
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

    const navMode = getSelectedNavMode();

    // Send config update first
    const config = {
        lanes: lanes,
        scan_length: length,
        scan_width: width,
        // Carry A* settings to Sputnik with waypoint generation
        astar_enabled: navMode !== 'simple',
        astar_hybrid_mode: navMode === 'hybrid',
        hazard_enabled: navMode === 'hybrid',
        astar_resolution: parseFloat(document.getElementById('cfg-astar-resolution').value),
        astar_safety_margin: parseFloat(document.getElementById('cfg-astar-safety').value),
        astar_max_expansions: parseInt(document.getElementById('cfg-astar-max').value)
    };
    
    const configMsg = new ROSLIB.Message({
        data: JSON.stringify(config)
    });
    configPublisher.publish(configMsg);
    
    // Also publish to modular sputnik planner
    if (modularConfigPublisher) {
        modularConfigPublisher.publish(configMsg);
    }
    
    // Then send generate command
    sendMissionCommand('generate_waypoints');
    
    // Calculate and display preview info
    const totalWaypoints = lanes * 2 - 1;
    const estimatedDistance = length * lanes + width * (lanes - 1);
    document.getElementById('waypoint-count').textContent = `Waypoints: ~${totalWaypoints}`;
    document.getElementById('estimated-distance').textContent = `Distance: ~${estimatedDistance}m`;
    
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

    // Ensure modular publisher exists too (avoid race after hard-refresh)
    if (!modularMissionCommandPublisher) {
        modularMissionCommandPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/sputnik/mission_command',
            messageType: 'std_msgs/String'
        });
    }
    
    const msg = new ROSLIB.Message({
        data: JSON.stringify({ command: command })
    });
    
    // Publish to both vostok1 and modular (sputnik) topics
    missionCommandPublisher.publish(msg);
    if (modularMissionCommandPublisher) {
        modularMissionCommandPublisher.publish(msg);
    }
    addLog(`Mission command: ${command}`, 'info');
    console.log('Mission command sent to both vostok1 and sputnik:', command);
}

// Update mission control UI based on state
function updateMissionControlUI(state) {
    console.log('updateMissionControlUI called with state:', state);
    missionState.state = state.state || 'IDLE';
    // If no gps_ready field, assume true when connected (for testing)
    missionState.gpsReady = (state.gps_ready !== undefined) ? state.gps_ready : connected;
    missionState.missionArmed = state.mission_armed || false;
    missionState.joystickOverride = state.joystick_override || false;
    missionState.totalWaypoints = (state.total_waypoints !== undefined)
        ? state.total_waypoints
        : missionState.totalWaypoints;
    missionState.currentWaypoint = state.current_waypoint || 0;
    missionState.startLat = state.start_lat;
    missionState.startLon = state.start_lon;
    if (missionState.waypoints && missionState.waypoints.length > 0) {
        persistWaypoints();
    }
    
    // Update GPS indicator
    const gpsBadge = document.getElementById('gps-ready-badge');
    if (gpsBadge) {
        if (missionState.gpsReady) {
            gpsBadge.textContent = '📡 GPS: Ready | Prêt';
            gpsBadge.className = 'gps-badge ready';
        } else {
            gpsBadge.textContent = '📡 GPS: Waiting... | En attente...';
            gpsBadge.className = 'gps-badge not-ready';
        }
    }

    // Update state badge
    const stateBadge = document.getElementById('mission-state-badge');
    if (stateBadge) {
        const stateLabels = {
            'INIT': 'Init | Initialisation',
            'IDLE': 'Idle | En attente',
            'WAITING_CONFIRM': 'Confirm | Confirmation',
            'WAYPOINTS_PREVIEW': 'Preview | Aperçu',
            'READY': 'Ready | Prêt',
            'RUNNING': 'Running | En cours',
            'DRIVING': 'Driving | Navigation',
            'PAUSED': 'Paused | Pause',
            'JOYSTICK': 'Joystick',
            'FINISHED': 'Finished | Terminé'
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
    const hasWaypoints = (missionState.totalWaypoints || 0) > 0 || (missionState.waypoints && missionState.waypoints.length > 0);
    
    // Reset all buttons
    [btnGenerate, btnConfirm, btnCancel, btnStart, btnStop, btnResume, btnJoyEnable, btnJoyDisable].forEach(btn => {
        if (btn) btn.disabled = true;
    });
    
    // Enable buttons based on state
    switch (missionState.state) {
        case 'INIT':  // Modular sputnik uses INIT instead of IDLE
        case 'IDLE':
            if (missionState.gpsReady) {
                btnGenerate.disabled = false;
            }
            btnJoyEnable.disabled = false;
            break;
        
        case 'WAITING_CONFIRM':  // Modular sputnik uses this instead of WAYPOINTS_PREVIEW
        case 'WAYPOINTS_PREVIEW':
            btnGenerate.disabled = false;
            btnConfirm.disabled = !hasWaypoints;
            btnCancel.disabled = !hasWaypoints;
            btnJoyEnable.disabled = false;
            break;
            
        case 'READY':
            btnStart.disabled = !hasWaypoints;
            btnCancel.disabled = !hasWaypoints;
            btnJoyEnable.disabled = false;
            break;
        
        case 'DRIVING':  // Modular sputnik uses DRIVING instead of RUNNING
        case 'RUNNING':
            btnStop.disabled = false;
            break;
            
        case 'PAUSED':
            btnResume.disabled = !hasWaypoints;
            btnStart.disabled = !hasWaypoints;
            btnCancel.disabled = !hasWaypoints;
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
    
    // Get reference point: use mission start if available, otherwise use current boat position
    let startLat = missionState.startLat;
    let startLon = missionState.startLon;
    
    if (!startLat || !startLon) {
        // Fallback: use boat's current GPS position as reference
        // This allows waypoints to display even if config hasn't arrived yet
        startLat = currentState.gps.lat;
        startLon = currentState.gps.lon;
        
        if (!startLat || !startLon || startLat === 0) {
            // Still no reference point, can't display waypoints yet
            return;
        }
    }
    
    // Convert local coordinates to GPS
    // Handle both formats: [{x, y}] from vostok1 or [[x, y]] from sputnik
    const waypointLatLngs = waypoints.map((wp, idx) => {
        const x = Array.isArray(wp) ? wp[0] : wp.x;
        const y = Array.isArray(wp) ? wp[1] : wp.y;
        const latLng = localToGPS(x, y, startLat, startLon);
        return { lat: latLng[0], lon: latLng[1], x: x, y: y, idx: idx };
    });
    
    // Create waypoint markers
    waypointLatLngs.forEach((wp, idx) => {
        // Note: missionState.currentWaypoint is 1-indexed (from planner), but idx is 0-indexed
        // Default to 0 if not yet initialized (after page refresh)
        const currentWpIndex = (missionState.currentWaypoint || 1) - 1;
        const isCurrentTarget = idx === currentWpIndex;
        const isPassed = idx < currentWpIndex;
        
        const markerColor = isPassed ? 'green' : (isCurrentTarget ? 'orange' : 'blue');
        const markerSize = isCurrentTarget ? 14 : 10;
        const statusText = isPassed ? '✓ Passed | Passé' : (isCurrentTarget ? '→ Target | Cible' : 'Pending | En attente');
        
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
                permanent: false,  // Only show on hover
                direction: 'top',
                offset: [0, -10],
                className: 'waypoint-tooltip-container',
                sticky: true  // Keeps tooltip visible while mouse hovers over waypoint
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

function clearPollutantMarkers() {
    pollutantMarkers.forEach(marker => map.removeLayer(marker));
    pollutantMarkers = [];
}

function updatePollutantSources(sources) {
    clearPollutantMarkers();
    if (!sources || sources.length === 0) return;

    // Need a reference GPS to convert local->GPS
    let refLat = missionState.startLat || currentState.gps.lat;
    let refLon = missionState.startLon || currentState.gps.lon;
    if (!refLat || !refLon || refLat === 0) return;

    sources.forEach(src => {
        const lx = src.local ? src.local[0] : 0.0;
        const ly = src.local ? src.local[1] : 0.0;
        const latLng = localToGPS(lx, ly, refLat, refLon);
        const icon = L.divIcon({
            className: 'pollutant-marker',
            html: `<div style="
                background: radial-gradient(circle, rgba(200,50,50,0.9) 0%, rgba(120,0,0,0.8) 60%);
                width: 14px; height: 14px; border-radius: 50%; border: 2px solid white;
                box-shadow: 0 0 6px rgba(0,0,0,0.6);
            "></div>`,
            iconSize: [14, 14],
            iconAnchor: [7, 7]
        });
        const marker = L.marker([latLng[0], latLng[1]], { icon: icon })
            .bindTooltip(
                `<strong>Pollutant</strong><br>${src.name || 'smoke'}<br>Local: (${lx.toFixed(1)}, ${ly.toFixed(1)})`,
                { direction: 'top', offset: [0, -8], permanent: false, sticky: true }
            )
            .addTo(map);
        pollutantMarkers.push(marker);
    });
}

// Clear stored waypoints + UI affordances
function clearWaypoints(reason = '') {
    missionState.waypoints = [];
    missionState.totalWaypoints = 0;
    missionState.currentWaypoint = 0;
    clearWaypointPreview();

    const wpCount = document.getElementById('waypoint-count');
    if (wpCount) wpCount.textContent = 'Waypoints: 0';
    const distEstimate = document.getElementById('estimated-distance');
    if (distEstimate) distEstimate.textContent = 'Distance: 0m';

    if (reason) {
        addLog(`Waypoints cleared (${reason})`, 'warning');
    }

    try {
        localStorage.removeItem(WAYPOINT_STORAGE_KEY);
    } catch (err) {
        console.warn('Failed to clear waypoint cache', err);
    }
}

function persistWaypoints() {
    try {
        const payload = {
            waypoints: missionState.waypoints || [],
            totalWaypoints: missionState.totalWaypoints || 0,
            startLat: missionState.startLat,
            startLon: missionState.startLon
        };
        localStorage.setItem(WAYPOINT_STORAGE_KEY, JSON.stringify(payload));
    } catch (err) {
        console.warn('Failed to persist waypoints', err);
    }
}

function restorePersistedWaypoints() {
    try {
        const raw = localStorage.getItem(WAYPOINT_STORAGE_KEY);
        if (!raw) return;
        const data = JSON.parse(raw);
        if (data.waypoints && data.waypoints.length > 0) {
            missionState.waypoints = data.waypoints;
            missionState.totalWaypoints = data.totalWaypoints || data.waypoints.length;
            missionState.startLat = data.startLat || missionState.startLat;
            missionState.startLon = data.startLon || missionState.startLon;
            addLog(`Restored ${missionState.waypoints.length} cached waypoints`, 'info');
            displayWaypointsOnMap(missionState.waypoints, true);
        }
    } catch (err) {
        console.warn('Failed to restore waypoint cache', err);
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
