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
let enablePublisher;
let autoGoalConfigPublisher;
let cancelGoalPublisher;

// Mission state
let lastGoal = null; // Store last goal for resume functionality

// Camera (no need for cameraUpdateInterval - MJPEG is continuous)

// Map
let map;
let boatMarker;
let goalMarker;
let previewGoalMarker; // Preview marker before confirming goal
let trajectoryLine;
let trajectoryPoints = [];
let mapFollowBoat = true;
let smokeMarkers = [];
let waypointMarkers = []; // For auto-goal waypoint sequence visualization
let latestLocalPose = null; // Latest local ENU pose (meters)

// Map grid overlay (local ENU meters)
let gridLayerGroup;
let gridEnabled = true;
let lastGridMinorSpacingM = null;

// Map origin (local ENU (0,0) -> GPS lat/lon)
// Default values correspond to the common VRX spawn pose [-532, 162] in sydney_regatta_smoke(.sdf)
// but we will auto-calibrate from the first GPS fix to avoid map drift between runs.
const DEFAULT_MAP_ORIGIN_LAT = -33.722776;
const DEFAULT_MAP_ORIGIN_LON = 150.673987;
let mapOriginLat = DEFAULT_MAP_ORIGIN_LAT;
let mapOriginLon = DEFAULT_MAP_ORIGIN_LON;
let mapOriginCalibrated = false;
let mapOriginCalibratedUsingPose = false;
let latestGpsFix = null;

// Approx conversions at ~-33.72° latitude (good enough for short distances)
const DEG_PER_M_LAT = 0.000009009;   // meters North -> degrees latitude
const DEG_PER_M_LON = 0.000010832;   // meters East  -> degrees longitude

function localToLatLon(localX, localY) {
    return [
        mapOriginLat + localY * DEG_PER_M_LAT,
        mapOriginLon + localX * DEG_PER_M_LON
    ];
}

function latLonToLocal(lat, lon) {
    return {
        x: (lon - mapOriginLon) / DEG_PER_M_LON,
        y: (lat - mapOriginLat) / DEG_PER_M_LAT
    };
}

function niceStepMeters(targetMeters) {
    if (!Number.isFinite(targetMeters) || targetMeters <= 0) return 50;
    const pow10 = Math.pow(10, Math.floor(Math.log10(targetMeters)));
    const mantissa = targetMeters / pow10;

    let niceMantissa = 1;
    if (mantissa <= 1) niceMantissa = 1;
    else if (mantissa <= 2) niceMantissa = 2;
    else if (mantissa <= 5) niceMantissa = 5;
    else niceMantissa = 10;

    return niceMantissa * pow10;
}

function computeGridMinorSpacingMeters() {
    if (!map) return 50;
    const size = map.getSize();
    const bounds = map.getBounds();
    if (!size || size.x <= 0) return 50;

    const centerLat = (bounds.getNorth() + bounds.getSouth()) / 2;
    const widthMeters = map.distance(
        L.latLng(centerLat, bounds.getWest()),
        L.latLng(centerLat, bounds.getEast())
    );

    const metersPerPixel = widthMeters / size.x;
    const targetPixels = 85;
    const targetMeters = metersPerPixel * targetPixels;
    return Math.max(1, niceStepMeters(targetMeters));
}

function updateGridLegend() {
    const legend = document.getElementById('grid-spacing-legend');
    const value = document.getElementById('grid-spacing-value');
    if (!legend || !value) return;

    if (!gridEnabled || !Number.isFinite(lastGridMinorSpacingM)) {
        legend.style.display = 'none';
        return;
    }

    legend.style.display = '';
    value.textContent = String(Math.round(lastGridMinorSpacingM));
}

function updateMapGrid() {
    if (!map) return;

    if (!gridEnabled) {
        if (gridLayerGroup) gridLayerGroup.clearLayers();
        lastGridMinorSpacingM = null;
        updateGridLegend();
        return;
    }

    if (!gridLayerGroup) {
        gridLayerGroup = L.layerGroup().addTo(map);
    }

    const bounds = map.getBounds();
    const { x: minX, y: minY } = latLonToLocal(bounds.getSouth(), bounds.getWest());
    const { x: maxX, y: maxY } = latLonToLocal(bounds.getNorth(), bounds.getEast());

    let minorSpacingM = computeGridMinorSpacingMeters();
    const majorEvery = 5;

    // Avoid rendering an excessive number of lines on wide zoom levels
    while (
        (Math.ceil(Math.abs(maxX - minX) / minorSpacingM) + 1) > 160 ||
        (Math.ceil(Math.abs(maxY - minY) / minorSpacingM) + 1) > 160
    ) {
        minorSpacingM *= 2;
    }

    gridLayerGroup.clearLayers();

    const startXi = Math.floor(Math.min(minX, maxX) / minorSpacingM);
    const endXi = Math.ceil(Math.max(minX, maxX) / minorSpacingM);
    const startYi = Math.floor(Math.min(minY, maxY) / minorSpacingM);
    const endYi = Math.ceil(Math.max(minY, maxY) / minorSpacingM);

    for (let xi = startXi; xi <= endXi; xi++) {
        const localX = xi * minorSpacingM;
        const isMajor = (((xi % majorEvery) + majorEvery) % majorEvery) === 0;
        const style = isMajor
            ? { color: '#ffffff', weight: 1.5, opacity: 0.25 }
            : { color: '#ffffff', weight: 1, opacity: 0.12, dashArray: '2,6' };

        const [lat1, lon1] = localToLatLon(localX, Math.min(minY, maxY));
        const [lat2, lon2] = localToLatLon(localX, Math.max(minY, maxY));
        L.polyline([[lat1, lon1], [lat2, lon2]], { ...style, pane: 'gridPane', interactive: false }).addTo(gridLayerGroup);
    }

    for (let yi = startYi; yi <= endYi; yi++) {
        const localY = yi * minorSpacingM;
        const isMajor = (((yi % majorEvery) + majorEvery) % majorEvery) === 0;
        const style = isMajor
            ? { color: '#ffffff', weight: 1.5, opacity: 0.25 }
            : { color: '#ffffff', weight: 1, opacity: 0.12, dashArray: '2,6' };

        const [lat1, lon1] = localToLatLon(Math.min(minX, maxX), localY);
        const [lat2, lon2] = localToLatLon(Math.max(minX, maxX), localY);
        L.polyline([[lat1, lon1], [lat2, lon2]], { ...style, pane: 'gridPane', interactive: false }).addTo(gridLayerGroup);
    }

    lastGridMinorSpacingM = minorSpacingM;
    updateGridLegend();
}

function maybeCalibrateMapOriginFromGps(latitude, longitude) {
    if (mapOriginCalibrated) return;

    // If we already have a local pose, align ENU pose (x,y) with GPS (lat,lon)
    if (latestLocalPose && Number.isFinite(latestLocalPose.x) && Number.isFinite(latestLocalPose.y)) {
        mapOriginLat = latitude - latestLocalPose.y * DEG_PER_M_LAT;
        mapOriginLon = longitude - latestLocalPose.x * DEG_PER_M_LON;
        mapOriginCalibratedUsingPose = true;
    } else {
        // Fallback: treat the first GPS fix as the local origin
        mapOriginLat = latitude;
        mapOriginLon = longitude;
        mapOriginCalibratedUsingPose = false;
    }

    mapOriginCalibrated = true;

    // Refresh markers that depend on origin (smoke markers are the main reason)
    addSmokeMarkers();
    if (lastGoal) {
        updateGoalMarker(lastGoal.x, lastGoal.y);
    }
    updateMapGrid();

    log(`Map origin calibrated from GPS: (${mapOriginLat.toFixed(6)}, ${mapOriginLon.toFixed(6)})`, 'info');
}

function maybeRefineMapOriginUsingPose() {
    if (!mapOriginCalibrated) return;
    if (mapOriginCalibratedUsingPose) return;
    if (!latestGpsFix) return;
    if (!latestLocalPose || !Number.isFinite(latestLocalPose.x) || !Number.isFinite(latestLocalPose.y)) return;

    mapOriginLat = latestGpsFix.lat - latestLocalPose.y * DEG_PER_M_LAT;
    mapOriginLon = latestGpsFix.lon - latestLocalPose.x * DEG_PER_M_LON;
    mapOriginCalibratedUsingPose = true;

    addSmokeMarkers();
    if (lastGoal) {
        updateGoalMarker(lastGoal.x, lastGoal.y);
    }
    updateMapGrid();

    log(`Map origin refined using pose+GPS: (${mapOriginLat.toFixed(6)}, ${mapOriginLon.toFixed(6)})`, 'info');
}

// Node logs subscriber
let nodeLogsSubscriber;

// ============================================================================
// Initialize Map
// ============================================================================
function initMap() {
    // Initialize map at real boat start point (actual coordinates from GPS)
    // Latitude: -33.722776, Longitude: 150.673987
    // Start with wider view (zoom 14) to see both boat spawn and smoke obstacles
    map = L.map('map').setView([mapOriginLat, mapOriginLon], 14);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap contributors'
    }).addTo(map);

    // Grid overlay for distance estimation (in local ENU meters)
    if (!map.getPane('gridPane')) {
        map.createPane('gridPane');
        map.getPane('gridPane').style.zIndex = 350;
        map.getPane('gridPane').style.pointerEvents = 'none';
    }

    // Custom boat icon
    const boatIcon = L.divIcon({
        className: 'boat-marker',
        html: '🚤',
        iconSize: [30, 30],
        iconAnchor: [15, 15]
    });

    boatMarker = L.marker([mapOriginLat, mapOriginLon], { icon: boatIcon }).addTo(map);
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

    // Add "Show All Markers" button
    const ShowAllControl = L.Control.extend({
        options: { position: 'topright' },
        onAdd: function(map) {
            const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');
            const button = L.DomUtil.create('a', 'control-btn', container);
            button.href = '#';
            button.title = 'Show all markers';
            button.innerHTML = '🗺️';
            button.style.fontSize = '18px';
            button.style.width = '34px';
            button.style.height = '34px';
            button.style.lineHeight = '34px';
            button.style.textAlign = 'center';
            button.style.display = 'block';

            L.DomEvent.on(button, 'click', function(e) {
                L.DomEvent.preventDefault(e);
                L.DomEvent.stopPropagation(e);

                // Collect all marker positions
                const allMarkers = [];
                if (boatMarker) allMarkers.push(boatMarker);
                if (goalMarker) allMarkers.push(goalMarker);
                smokeMarkers.forEach(m => allMarkers.push(m));
                waypointMarkers.forEach(m => allMarkers.push(m));

                if (allMarkers.length > 0) {
                    const group = L.featureGroup(allMarkers);
                    map.fitBounds(group.getBounds().pad(0.1));
                    log('Map zoomed to show all markers', 'info');
                }
            });

            return container;
        }
    });

    map.addControl(new ShowAllControl());

    // Add grid toggle button
    const GridToggleControl = L.Control.extend({
        options: { position: 'topright' },
        onAdd: function(map) {
            const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control grid-control');
            const button = L.DomUtil.create('a', 'grid-toggle-btn', container);
            button.href = '#';
            button.title = 'Toggle grid';
            button.innerHTML = '▦';
            button.id = 'grid-toggle';

            L.DomEvent.on(button, 'click', function(e) {
                L.DomEvent.preventDefault(e);
                L.DomEvent.stopPropagation(e);
                gridEnabled = !gridEnabled;
                button.className = gridEnabled ? 'grid-toggle-btn active' : 'grid-toggle-btn';
                updateMapGrid();
            });

            button.className = gridEnabled ? 'grid-toggle-btn active' : 'grid-toggle-btn';
            return container;
        }
    });

    map.addControl(new GridToggleControl());

    // Add smoke obstacle markers (sydney_regatta_smoke world)
    addSmokeMarkers();

    updateMapGrid();
    map.on('moveend zoomend', updateMapGrid);

    log('Map initialized', 'info');
}

// Add smoke obstacle markers from sydney_regatta_smoke.sdf
function addSmokeMarkers() {
    if (!map) return;

    // Smoke generator coordinates from uvautoboat/test_environment/sydney_regatta_smoke.sdf
    // Converted to local ENU (meters) by subtracting the VRX default spawn pose [-532, 162].
    // Format: [localX, localY, name]
    const smokeLocations = [
        [-3, 98, 'Smoke Generator (Main)'],
        [32, 148, 'Smoke Generator (East)'],
        [-48, 178, 'Smoke Generator (North)']
    ];

    // Clear existing smoke markers to avoid duplicates when re-calibrating origin
    smokeMarkers.forEach(marker => map.removeLayer(marker));
    smokeMarkers = [];

    smokeLocations.forEach(([localX, localY, name]) => {
        // Convert local ENU coordinates to GPS lat/lon
        const [smokeLat, smokeLon] = localToLatLon(localX, localY);

        console.log(`Adding smoke marker: ${name} at local (${localX}, ${localY}) -> GPS (${smokeLat}, ${smokeLon})`);

        // Create smoke marker with hazard icon
        const smokeIcon = L.divIcon({
            className: 'smoke-marker',
            html: '☁️',
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });

        const marker = L.marker([smokeLat, smokeLon], { icon: smokeIcon }).addTo(map);
        marker.bindPopup(`<b>⚠️ ${name}</b><br>World: (${localX}m, ${localY}m)<br><i>Smoke/pollution hazard</i>`);
        smokeMarkers.push(marker);
    });

    log(`Smoke obstacle markers added to map: ${smokeMarkers.length} markers`, 'info');
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
        messageType: 'std_msgs/Float64'
    });

    thrustLeftSubscriber.subscribe(function(message) {
        updateThrustDisplay('left', message.data);
    });

    // Subscribe to right thruster
    thrustRightSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/thrusters/right/thrust',
        messageType: 'std_msgs/Float64'
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

    // Subscribe to smoke detection
    const smokeDetectionSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/perception/smoke_detected',
        messageType: 'std_msgs/String'
    });

    smokeDetectionSubscriber.subscribe(function(message) {
        try {
            const smokeData = JSON.parse(message.data);
            updateSmokeDetection(smokeData);
        } catch (e) {
            console.error('Error parsing smoke detection data:', e);
        }
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
        // Debug: Log the message structure to console (comment out after debugging)
        console.log('Rosout message:', message);

        // Only show logs from robust_avoidance node
        // The 'name' field in rcl_interfaces/Log contains the node name without leading slash
        if (message.name === 'robust_avoidance' || message.name === '/robust_avoidance') {
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

    enablePublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/robust_avoidance/enable',
        messageType: 'std_msgs/Bool'
    });

    autoGoalConfigPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/robust_avoidance/auto_goal_config',
        messageType: 'std_msgs/String'
    });

    cancelGoalPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/robust_avoidance/cancel_goal',
        messageType: 'std_msgs/Bool'
    });
}

// ============================================================================
// Update Display Functions
// ============================================================================
function updatePoseDisplay(message) {
    const x = message.pose.position.x;
    const y = message.pose.position.y;
    latestLocalPose = { x, y };
    maybeRefineMapOriginUsingPose();

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

function updateSmokeDetection(smokeData) {
    const smokeStatus = document.getElementById('smoke-status');
    const smokeDistance = document.getElementById('smoke-distance');
    const smokePoints = document.getElementById('smoke-points');
    const smokePosition = document.getElementById('smoke-position');

    if (smokeData.detected) {
        smokeStatus.textContent = '🌫️ SMOKE DETECTED';
        smokeStatus.className = 'value badge warning';
        smokeDistance.textContent = smokeData.distance.toFixed(1) + ' m';
        smokePoints.textContent = smokeData.point_count;
        smokePosition.textContent = `(${smokeData.center_x.toFixed(1)}, ${smokeData.center_y.toFixed(1)})`;
    } else {
        smokeStatus.textContent = 'No Smoke';
        smokeStatus.className = 'value badge clear';
        smokeDistance.textContent = '-';
        smokePoints.textContent = '-';
        smokePosition.textContent = '-';
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

    // Calibrate map origin once we receive GPS (reduces drift vs hardcoded origin)
    latestGpsFix = { lat: latitude, lon: longitude };
    maybeCalibrateMapOriginFromGps(latitude, longitude);

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

function previewGoal() {
    if (!map) {
        alert('❌ Map is still initializing. Please wait and try again.');
        log('Cannot preview goal: map not initialized', 'error');
        return;
    }

    const x = parseFloat(document.getElementById('goal-x').value);
    const y = parseFloat(document.getElementById('goal-y').value);

    if (isNaN(x) || isNaN(y)) {
        alert('❌ Invalid goal coordinates. Please enter numeric X/Y values.');
        log('Cannot preview goal: invalid coordinates provided', 'error');
        return;
    }

    // Remove existing preview marker
    if (previewGoalMarker) {
        map.removeLayer(previewGoalMarker);
    }

    // Convert to GPS coordinates
    const [previewLat, previewLon] = localToLatLon(x, y);

    // Create preview marker (different style from actual goal)
    const previewIcon = L.divIcon({
        className: 'preview-goal-marker',
        html: '<div style="background: orange; color: white; border-radius: 50%; width: 30px; height: 30px; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 3px dashed white; animation: pulse 1.5s infinite;">?</div>',
        iconSize: [30, 30],
        iconAnchor: [15, 15]
    });

    previewGoalMarker = L.marker([previewLat, previewLon], { icon: previewIcon }).addTo(map);
    previewGoalMarker.bindPopup(`<b>📍 Goal Preview</b><br>Local: (${x.toFixed(1)}m, ${y.toFixed(1)}m)<br><i>Click "Confirm & Send Goal" to navigate here</i>`).openPopup();

    // Pan to preview marker
    map.panTo([previewLat, previewLon]);

    // Show clear button
    document.getElementById('btn-clear-preview').style.display = 'block';

    log(`Goal preview: (${x.toFixed(1)}, ${y.toFixed(1)}) - Review on map`, 'info');
}

function clearPreview() {
    if (previewGoalMarker) {
        map.removeLayer(previewGoalMarker);
        previewGoalMarker = null;
    }

    // Hide clear button
    document.getElementById('btn-clear-preview').style.display = 'none';

    log('Goal preview cleared', 'info');
}

function sendGoal() {
    if (!connected) {
        alert('❌ Cannot send goal: Not connected to ROS');
        log('Cannot send goal: Not connected to ROS', 'error');
        return;
    }

    const x = parseFloat(document.getElementById('goal-x').value);
    const y = parseFloat(document.getElementById('goal-y').value);

    if (isNaN(x) || isNaN(y)) {
        alert('❌ Invalid goal coordinates. Please enter numeric X/Y values.');
        log('Cannot send goal: invalid coordinates provided', 'error');
        return;
    }

    // Confirm goal before sending (if preview exists, mention it)
    let confirmMsg = `📍 Confirm Goal?\n\nTarget: (${x.toFixed(1)}m, ${y.toFixed(1)}m)\n\n`;
    if (previewGoalMarker) {
        confirmMsg += `Preview marker is visible on map.\n\n`;
    }
    confirmMsg += `The boat will navigate to this position.\n\nContinue?`;

    const confirmed = confirm(confirmMsg);

    if (!confirmed) {
        log('Goal cancelled by user', 'warning');
        return;
    }

    // Clear preview if exists
    clearPreview();

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

    // Store goal for resume functionality
    lastGoal = { x: x, y: y };

    // Add goal marker on map
    updateGoalMarker(x, y);

    alert(`✅ Goal Set!\n\nNavigating to: (${x.toFixed(1)}m, ${y.toFixed(1)}m)`);
    log('Goal sent: (' + x + ', ' + y + ')', 'success');
}

function cancelGoal() {
    if (!connected) {
        alert('❌ Cannot cancel goal: Not connected to ROS');
        log('Cannot cancel goal: Not connected to ROS', 'error');
        return;
    }

    const hasGoal = goalMarker !== null || lastGoal !== null;
    if (!hasGoal) {
        alert('ℹ️ No active goal to cancel.');
        log('Cancel goal ignored: no active goal present', 'warning');
        return;
    }

    const confirmed = confirm('✗ Cancel Current Goal?\n\nThis will stop navigation and clear the active path.\n\nContinue?');
    if (!confirmed) {
        log('Goal cancellation aborted by user', 'warning');
        return;
    }

    cancelGoalPublisher.publish(new ROSLIB.Message({ data: true }));

    if (goalMarker) {
        map.removeLayer(goalMarker);
        goalMarker = null;
    }
    lastGoal = null;
    clearPreview();

    alert('✗ Goal Cancelled\n\nController instructed to stop and clear the active goal.');
    log('Goal cancellation sent to controller', 'warning');
}

function stopMission() {
    if (!connected) {
        alert('❌ Not connected to ROS');
        log('Cannot stop: Not connected to ROS', 'error');
        return;
    }

    // Disable the controller - this will stop thrust and keep it stopped
    enablePublisher.publish(new ROSLIB.Message({ data: false }));
    alert('⏸ Mission Stopped\n\nController disabled, thrusters stopped.');
    log('Mission stopped - controller disabled', 'warning');
}

function resumeMission() {
    if (!connected) {
        alert('❌ Not connected to ROS');
        log('Cannot resume: Not connected to ROS', 'error');
        return;
    }

    // Re-enable the controller - it will continue with the current goal
    enablePublisher.publish(new ROSLIB.Message({ data: true }));
    alert('▶ Mission Resumed\n\nController re-enabled, continuing navigation.');
    log('Mission resumed - controller re-enabled', 'success');
}

function resetMission() {
    if (!connected) {
        alert('❌ Not connected to ROS');
        log('Cannot reset: Not connected to ROS', 'error');
        return;
    }

    const confirmed = confirm('🔄 Reset Mission?\n\nThis will:\n- Disable the controller\n- Clear the current goal\n- Stop all movement\n\nContinue?');

    if (!confirmed) {
        log('Reset cancelled by user', 'warning');
        return;
    }

    // Disable controller and clear stored goal
    enablePublisher.publish(new ROSLIB.Message({ data: false }));
    lastGoal = null;

    // Remove goal marker from map
    if (goalMarker) {
        map.removeLayer(goalMarker);
        goalMarker = null;
    }

    alert('🔄 Mission Reset\n\nController disabled and goal cleared.');
    log('Mission reset - controller disabled and goal cleared', 'info');
}

function goHome() {
    if (!connected) {
        alert('❌ Not connected to ROS');
        log('Cannot go home: Not connected to ROS', 'error');
        return;
    }

    if (confirm('🏠 Go Home?\n\nThe boat will navigate back to origin (0, 0).\n\nContinue?')) {
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
                    x: 0.0,
                    y: 0.0,
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
        lastGoal = { x: 0.0, y: 0.0 };

        // Update goal marker
        updateGoalMarker(0.0, 0.0);

        alert('🏠 Go Home Activated\n\nNavigating to origin (0, 0)');
        log('🏠 Go Home activated - navigating to (0, 0)', 'info');
    }
}

function emergencyStop() {
    if (!connected) {
        alert('❌ Not connected to ROS');
        log('Cannot stop: Not connected to ROS', 'error');
        return;
    }

    const confirmed = confirm('🚨 EMERGENCY STOP 🚨\n\nThis will IMMEDIATELY:\n- Disable the controller\n- Cut all thrust to ZERO\n- Stop the boat\n\nConfirm emergency stop?');

    if (!confirmed) {
        log('Emergency stop cancelled by user', 'warning');
        return;
    }

    log('🚨 EMERGENCY STOP ACTIVATED 🚨', 'error');

    // Disable controller multiple times to ensure it's received
    for (let i = 0; i < 5; i++) {
        setTimeout(() => {
            enablePublisher.publish(new ROSLIB.Message({ data: false }));
        }, i * 100); // 0ms, 100ms, 200ms, 300ms, 400ms
    }

    alert('🚨 EMERGENCY STOP ACTIVATED\n\nController disabled, all thrust stopped.\nUse Resume to re-enable.');
    log('Controller disabled - all thrust stopped', 'error');
    log('Boat should be stopped. Use Resume to re-enable.', 'warning');
}

// ============================================================================
// Auto-Goal Control Functions
// ============================================================================

function enableAutoGoal() {
    if (!connected) {
        alert('❌ Not connected to ROS');
        log('Cannot enable auto-goal: Not connected to ROS', 'error');
        return;
    }

    const goalsInput = document.getElementById('auto-goal-sequence').value.trim();
    if (!goalsInput) {
        alert('❌ No waypoints specified\n\nPlease enter waypoints in format: x1,y1;x2,y2;x3,y3');
        log('Cannot enable auto-goal: No waypoints specified', 'error');
        return;
    }

    // Validate goal format
    const goalPairs = goalsInput.split(';');
    let valid = true;
    for (const pair of goalPairs) {
        const parts = pair.trim().split(',');
        if (parts.length !== 2 || isNaN(parseFloat(parts[0])) || isNaN(parseFloat(parts[1]))) {
            valid = false;
            break;
        }
    }

    if (!valid) {
        alert('❌ Invalid waypoint format\n\nUse format: x1,y1;x2,y2;x3,y3\nExample: 130,0;130,50;80,60');
        log('Cannot enable auto-goal: Invalid waypoint format', 'error');
        return;
    }

    const confirmed = confirm(`▶ Enable Auto-Goal Mode?\n\nWaypoints: ${goalsInput}\n\nThe boat will automatically navigate through ${goalPairs.length} waypoints in sequence.\n\nContinue?`);

    if (!confirmed) {
        log('Auto-goal enable cancelled by user', 'warning');
        return;
    }

    // Send auto-goal configuration
    const config = {
        enable: true,
        goals: goalsInput
    };

    const configMessage = new ROSLIB.Message({
        data: JSON.stringify(config)
    });

    autoGoalConfigPublisher.publish(configMessage);

    // Visualize waypoints on map
    showWaypointSequence(goalsInput);

    // Update UI
    const statusBadge = document.getElementById('auto-goal-status-badge');
    statusBadge.textContent = 'Enabled';
    statusBadge.className = 'value badge success';

    alert(`✅ Auto-Goal Mode Enabled\n\nWaypoints: ${goalsInput}\n\nThe boat will navigate through ${goalPairs.length} waypoints automatically.\n\nWaypoints are now visible on the map.`);
    log(`Auto-goal mode enabled with ${goalPairs.length} waypoints: ${goalsInput}`, 'success');
}

function disableAutoGoal() {
    if (!connected) {
        alert('❌ Not connected to ROS');
        log('Cannot disable auto-goal: Not connected to ROS', 'error');
        return;
    }

    const confirmed = confirm('⏹ Disable Auto-Goal Mode?\n\nThis will stop automatic waypoint navigation.\n\nContinue?');

    if (!confirmed) {
        log('Auto-goal disable cancelled by user', 'warning');
        return;
    }

    // Send auto-goal configuration
    const config = {
        enable: false,
        goals: ''
    };

    const configMessage = new ROSLIB.Message({
        data: JSON.stringify(config)
    });

    autoGoalConfigPublisher.publish(configMessage);

    // Clear waypoint markers from map
    clearWaypointMarkers();

    // Update UI
    const statusBadge = document.getElementById('auto-goal-status-badge');
    statusBadge.textContent = 'Disabled';
    statusBadge.className = 'value badge';

    alert('⏹ Auto-Goal Mode Disabled\n\nAutomatic waypoint navigation stopped.\n\nWaypoint markers removed from map.');
    log('Auto-goal mode disabled', 'warning');
}

// Update goal marker on map (local coordinates to GPS approximate)
function updateGoalMarker(localX, localY) {
    if (!map) return;

    const [goalLat, goalLon] = localToLatLon(localX, localY);

    // Remove existing goal marker if present
    if (goalMarker) {
        map.removeLayer(goalMarker);
    }

    // Create goal marker
    const goalIcon = L.divIcon({
        className: 'goal-marker',
        html: '🎯',
        iconSize: [30, 30],
        iconAnchor: [15, 15]
    });

    goalMarker = L.marker([goalLat, goalLon], { icon: goalIcon }).addTo(map);
    goalMarker.bindPopup(`<b>Goal</b><br>Local: (${localX.toFixed(1)}m, ${localY.toFixed(1)}m)`);

    log(`Goal marker placed at approx (${goalLat.toFixed(6)}, ${goalLon.toFixed(6)})`, 'info');
}

// Visualize waypoint sequence on map
function showWaypointSequence(goalsString) {
    if (!map) return;

    // Clear existing waypoint markers
    clearWaypointMarkers();

    const goalPairs = goalsString.split(';');
    goalPairs.forEach((pair, index) => {
        const parts = pair.trim().split(',');
        if (parts.length === 2) {
            const localX = parseFloat(parts[0]);
            const localY = parseFloat(parts[1]);

            const [waypointLat, waypointLon] = localToLatLon(localX, localY);

            // Create numbered waypoint marker
            const waypointIcon = L.divIcon({
                className: 'waypoint-marker',
                html: `<div style="background: #3498db; color: white; border-radius: 50%; width: 24px; height: 24px; display: flex; align-items: center; justify-content: center; font-weight: bold; font-size: 12px; border: 2px solid white;">${index + 1}</div>`,
                iconSize: [24, 24],
                iconAnchor: [12, 12]
            });

            const marker = L.marker([waypointLat, waypointLon], { icon: waypointIcon }).addTo(map);
            marker.bindPopup(`<b>Waypoint ${index + 1}</b><br>Local: (${localX.toFixed(1)}m, ${localY.toFixed(1)}m)`);
            waypointMarkers.push(marker);
        }
    });

    log(`Waypoint sequence visualized: ${waypointMarkers.length} waypoints`, 'info');
}

// Clear waypoint markers from map
function clearWaypointMarkers() {
    if (!map) return;
    waypointMarkers.forEach(marker => map.removeLayer(marker));
    waypointMarkers = [];
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
    document.getElementById('btn-preview-goal').addEventListener('click', previewGoal);
    document.getElementById('btn-send-goal').addEventListener('click', sendGoal);
    document.getElementById('btn-cancel-goal').addEventListener('click', cancelGoal);
    document.getElementById('btn-clear-preview').addEventListener('click', clearPreview);
    document.getElementById('btn-clear-logs').addEventListener('click', clearLogs);
    document.getElementById('btn-clear-node-logs').addEventListener('click', clearNodeLogs);
    document.getElementById('btn-refresh-camera').addEventListener('click', function() {
        updateCameraFeed();
    });

    // Mission control buttons
    document.getElementById('btn-stop-mission').addEventListener('click', stopMission);
    document.getElementById('btn-resume-mission').addEventListener('click', resumeMission);
    document.getElementById('btn-reset-mission').addEventListener('click', resetMission);
    document.getElementById('btn-go-home').addEventListener('click', goHome);
    document.getElementById('btn-emergency-stop').addEventListener('click', emergencyStop);

    // Auto-goal control buttons
    document.getElementById('btn-enable-auto-goal').addEventListener('click', enableAutoGoal);
    document.getElementById('btn-disable-auto-goal').addEventListener('click', disableAutoGoal);
});
