// Path Predictor Convoy Follower Integration
// This script integrates ArcGIS map routing with drone convoy following

// 1. Add the necessary styles
function addPathPredictorStyles() {
  const styleTag = document.createElement('style');
  styleTag.innerHTML = `
    /* Path Predictor Styles */
    #path-predictor-popup {
      width: 400px;
      background-color: rgba(0, 0, 0, 0.8);
      border: 1px solid #00ff00;
      border-radius: 10px;
      box-shadow: 0 0 20px rgba(0, 255, 0, 0.3);
      color: white;
      font-family: 'Arial', sans-serif;
      padding: 20px;
      text-align: center;
      z-index: 1000;
    }

    #path-predictor-popup h3 {
      font-size: 1.8em;
      margin-top: 0;
      margin-bottom: 15px;
      color: #00ff00;
      text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5);
    }

    .path-predictor-section {
      margin-bottom: 15px;
      text-align: left;
    }

    .path-predictor-controls {
      display: flex;
      justify-content: space-between;
      flex-wrap: wrap;
      margin-bottom: 15px;
    }

    .path-predictor-controls button {
      margin: 5px;
      padding: 8px 12px;
      background-color: rgba(76, 175, 80, 0.8);
      color: white;
      border: none;
      border-radius: 4px;
      cursor: pointer;
    }

    .path-predictor-controls button:hover {
      background-color: rgba(76, 175, 80, 1);
    }

    .path-predictor-controls button.active {
      background-color: #ff9800;
    }

    .path-predictor-controls button.execute {
      background-color: #4CAF50;
      width: 100%;
      margin-top: 10px;
      font-weight: bold;
    }

    .path-predictor-info {
      background-color: rgba(0, 0, 0, 0.5);
      padding: 10px;
      border-radius: 5px;
      margin-bottom: 15px;
    }

    /* Path Point Styles */
    .path-point-marker {
      background-color: #0078A8;
      color: white;
      border-radius: 50%;
      width: 20px;
      height: 20px;
      text-align: center;
      line-height: 20px;
      font-weight: bold;
      border: 2px solid white;
      box-shadow: 0 0 4px rgba(0,0,0,0.4);
    }

    .drone-waypoint-marker {
      background-color: #FF4500;
      color: white;
      border-radius: 50%;
      width: 20px;
      height: 20px;
      text-align: center;
      line-height: 20px;
      font-weight: bold;
      border: 2px solid white;
      box-shadow: 0 0 4px rgba(0,0,0,0.4);
    }

    .distance-label {
      background-color: rgba(255, 255, 255, 0.8);
      border: 1px solid #FF4500;
      padding: 2px 5px;
      border-radius: 3px;
      font-size: 11px;
      font-weight: bold;
      white-space: nowrap;
      box-shadow: 0 0 3px rgba(0,0,0,0.2);
    }
  `;
  document.head.appendChild(styleTag);
}

// 2. Add the Path Predictor button to the GPS controls section
function addPathPredictorButton() {
  const gpsControls = document.getElementById('gps-controls');
  if (!gpsControls) {
    console.error('GPS controls section not found');
    return;
  }

  const buttonHtml = `
    <button class="button" id="path-predictor-btn" onclick="togglePathPredictorMode()">
      <i class="fas fa-route"></i> Path Predictor Convoy
    </button>
  `;

  gpsControls.insertAdjacentHTML('beforeend', buttonHtml);
  console.log('Path Predictor button added to GPS controls');
}

// 3. Create the Path Predictor popup
function createPathPredictorPopup() {
  // Check if popup already exists
  if (document.getElementById('path-predictor-popup')) {
    return;
  }

  // Create popup HTML
  const popupHtml = `
    <div id="path-predictor-popup" class="popup" style="display: none;">
      <h3>Path Predictor Convoy</h3>
      
      <div class="path-predictor-section">
        <p>Set start and end points to calculate the best route for convoy following.</p>
      </div>
      
      <div class="path-predictor-controls">
        <button id="path-start-btn" onclick="setPathStartPoint()">Set Start Point</button>
        <button id="path-end-btn" onclick="setPathEndPoint()">Set End Point</button>
        <button id="calculate-route-btn" onclick="calculateConvoyRoute()" disabled>Calculate Route</button>
        <button id="clear-route-btn" onclick="clearConvoyRoute()">Clear Route</button>
      </div>
      
      <div class="path-predictor-info">
        <div><strong>Distance:</strong> <span id="path-distance">-</span></div>
        <div><strong>Drone Path Distance:</strong> <span id="drone-path-distance">-</span></div>
        <div><strong>Travel Time:</strong> <span id="path-time">-</span></div>
        <div><strong>Road Type:</strong> <span id="path-road-type">-</span></div>
        <div><strong>Waypoints:</strong> <span id="path-waypoints">-</span></div>
      </div>
      
      <div class="path-predictor-controls">
        <button id="execute-path-mission-btn" class="execute" onclick="executePathMission()" disabled>Execute Convoy Mission</button>
      </div>
      
      <button onclick="closePathPredictorPopup()" class="button cancel-btn">
        <i class="fas fa-times"></i> Close
      </button>
    </div>
  `;

  // Add popup to document body
  document.body.insertAdjacentHTML('beforeend', popupHtml);
  console.log('Path Predictor popup created');
}

// 4. Load OSRM-related scripts and initialize the Path Predictor
function initPathPredictor() {
  console.log('Initializing Path Predictor...');

  // Initialize state variables
  window.pathPredictorState = {
    isActive: false,
    startMarker: null,
    endMarker: null,
    routePolyline: null,
    dronePathPolyline: null,
    droneWaypoints: [],
    droneWaypointMarkers: [],
    distanceLabels: []
  };
}

// 5. Path Predictor toggle function
function togglePathPredictorMode() {
  const state = window.pathPredictorState;
  state.isActive = !state.isActive;

  const pathPredictorBtn = document.getElementById('path-predictor-btn');
  const pathPredictorPopup = document.getElementById('path-predictor-popup');

  if (state.isActive) {
    // Enable mode
    pathPredictorBtn.classList.add('active');
    pathPredictorBtn.innerHTML = '<i class="fas fa-route"></i> Path Predictor (Active)';
    pathPredictorPopup.style.display = 'block';
    addImportantMessage("Path Predictor Mode: Enabled. Set start and end points for the convoy route.");
  } else {
    // Disable mode
    pathPredictorBtn.classList.remove('active');
    pathPredictorBtn.innerHTML = '<i class="fas fa-route"></i> Path Predictor Convoy';
    pathPredictorPopup.style.display = 'none';
    addImportantMessage("Path Predictor Mode: Disabled");
  }

  console.log("Path Predictor mode:", state.isActive ? "ENABLED" : "DISABLED");
}

// 6. Close Path Predictor popup
function closePathPredictorPopup() {
  const state = window.pathPredictorState;
  state.isActive = false;

  const pathPredictorBtn = document.getElementById('path-predictor-btn');
  const pathPredictorPopup = document.getElementById('path-predictor-popup');

  pathPredictorBtn.classList.remove('active');
  pathPredictorBtn.innerHTML = '<i class="fas fa-route"></i> Path Predictor Convoy';
  pathPredictorPopup.style.display = 'none';

  console.log("Path Predictor popup closed");
}

// 7. Set path start point
function setPathStartPoint() {
  const state = window.pathPredictorState;

  // Remove existing start marker if any
  if (state.startMarker) {
    map.removeLayer(state.startMarker);
  }

  // Get drone's current position if available, otherwise use map center
  let startPosition;
  if (droneMarker) {
    startPosition = droneMarker.getLatLng();
  } else {
    startPosition = map.getCenter();
  }

  // Create a new start marker
  const startIcon = L.icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-green.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
    shadowSize: [41, 41]
  });

  state.startMarker = L.marker(startPosition, {
    icon: startIcon,
    draggable: true
  }).addTo(map);

  state.startMarker.bindPopup("Start Point - Drag to adjust").openPopup();

  // Update buttons state
  updatePathPredictorButtonsState();

  addImportantMessage("Start point set. Now set an end point or drag the marker to adjust.");
}

// 8. Set path end point
function setPathEndPoint() {
  const state = window.pathPredictorState;

  // Remove existing end marker if any
  if (state.endMarker) {
    map.removeLayer(state.endMarker);
  }

  // Get a position 1km east of the start point, or map center if no start point
  let endPosition;
  if (state.startMarker) {
    const startPos = state.startMarker.getLatLng();
    // Move ~1km east (rough approximation)
    endPosition = L.latLng(startPos.lat, startPos.lng + 0.01);
  } else {
    endPosition = map.getCenter();
  }

  // Create a new end marker
  const endIcon = L.icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
    shadowSize: [41, 41]
  });

  state.endMarker = L.marker(endPosition, {
    icon: endIcon,
    draggable: true
  }).addTo(map);

  state.endMarker.bindPopup("End Point - Drag to adjust").openPopup();

  // Update buttons state
  updatePathPredictorButtonsState();

  addImportantMessage("End point set. You can now calculate the route or drag markers to adjust.");
}

// 9. Update buttons state based on markers
function updatePathPredictorButtonsState() {
  const state = window.pathPredictorState;
  const calculateRouteBtn = document.getElementById('calculate-route-btn');

  calculateRouteBtn.disabled = !(state.startMarker && state.endMarker);
}

// 10. Calculate the convoy route
function calculateConvoyRoute() {
  const state = window.pathPredictorState;

  if (!state.startMarker || !state.endMarker) {
    addImportantMessage("Please set both start and end points first.");
    return;
  }

  const startPoint = state.startMarker.getLatLng();
  const endPoint = state.endMarker.getLatLng();

  // Show loading indicator
  document.getElementById('path-distance').textContent = "Calculating...";
  document.getElementById('drone-path-distance').textContent = "Calculating...";
  document.getElementById('path-time').textContent = "Calculating...";
  document.getElementById('path-road-type').textContent = "Analyzing...";
  document.getElementById('path-waypoints').textContent = "Calculating...";

  // Clear any existing routes
  clearRouteVisuals();

  // Build API URL using OSRM public API
  const apiUrl = `https://router.project-osrm.org/route/v1/driving/${startPoint.lng},${startPoint.lat};${endPoint.lng},${endPoint.lat}?overview=full&geometries=polyline&steps=true`;

  // Make API request
  fetch(apiUrl)
    .then(response => {
      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }
      return response.json();
    })
    .then(data => {
      if (data.code !== 'Ok') {
        throw new Error(data.message || 'OSRM API error');
      }

      // Process the route
      processRouteData(data);
    })
    .catch(error => {
      console.error('Error calculating route:', error);

      // Show error message
      addImportantMessage("Route calculation failed: " + error.message);

      document.getElementById('path-distance').textContent = "Error";
      document.getElementById('drone-path-distance').textContent = "Error";
      document.getElementById('path-time').textContent = "Error";
      document.getElementById('path-road-type').textContent = error.message;
      document.getElementById('path-waypoints').textContent = "0";

      // Create a straight line between points as fallback
      const straightLine = L.polyline([startPoint, endPoint], {
        color: 'red',
        weight: 4,
        opacity: 0.7,
        dashArray: '5, 10'
      }).addTo(map);

      state.routePolyline = straightLine;

      // Calculate straight-line distance
      const distance = calculateHaversineDistance(
        startPoint.lat, startPoint.lng,
        endPoint.lat, endPoint.lng
      );

      document.getElementById('path-distance').textContent = distance.toFixed(2) + " km (direct)";
    });
}

// 11. Process route data and visualize
function processRouteData(data) {
  const state = window.pathPredictorState;
  const route = data.routes[0];

  // Decode polyline
  const coordinates = decodePolyline(route.geometry);

  // Create polyline for the route
  state.routePolyline = L.polyline(coordinates, {
    color: '#0078A8',
    weight: 6,
    opacity: 0.7
  }).addTo(map);

  // Analyze road curvature
  const { description, curveSegments } = analyzeRouteCurvature(coordinates);

  // Calculate drone path (simplified path for the drone to follow)
  const dronePathPoints = calculateDronePath(coordinates, 200); // 200 meters visibility

  // Create drone path polyline
  state.dronePathPolyline = L.polyline(dronePathPoints, {
    color: '#FF4500',
    weight: 4,
    opacity: 0.8
  }).addTo(map);

  // Store the drone waypoints
  state.droneWaypoints = dronePathPoints;

  // Highlight curve segments on the map
  highlightCurveSegments(curveSegments);

  // Update route info
  const distance = route.distance / 1000; // Convert to km
  const duration = route.duration / 60; // Convert to minutes

  document.getElementById('path-distance').textContent = distance.toFixed(2) + " km";

  // Calculate and display drone path distance
  let optimizedDistance = 0;
  for (let i = 0; i < dronePathPoints.length - 1; i++) {
    optimizedDistance += calculateHaversineDistance(
      dronePathPoints[i][0], dronePathPoints[i][1],
      dronePathPoints[i+1][0], dronePathPoints[i+1][1]
    );
  }
  document.getElementById('drone-path-distance').textContent = optimizedDistance.toFixed(2) + " km";

  // Format duration
  const hours = Math.floor(duration / 60);
  const minutes = Math.round(duration % 60);
  let timeText = "";
  if (hours > 0) {
    timeText += hours + " hr ";
  }
  timeText += minutes + " min";
  document.getElementById('path-time').textContent = timeText;

  document.getElementById('path-road-type').textContent = description;
  document.getElementById('path-waypoints').textContent = dronePathPoints.length + " waypoints";

  // Add numbered markers at each waypoint
  addWaypointMarkers(dronePathPoints, coordinates);

  // Fit map to the route
  map.fitBounds(state.routePolyline.getBounds(), { padding: [50, 50] });

  // Enable execute mission button
  document.getElementById('execute-path-mission-btn').disabled = false;

  addImportantMessage("Route calculated successfully. Review and press Execute to start the mission.");
}

// 12. Add waypoint markers along the route
function addWaypointMarkers(dronePathPoints, routeCoordinates) {
  const state = window.pathPredictorState;

  // Clear existing markers
  state.droneWaypointMarkers.forEach(marker => map.removeLayer(marker));
  state.droneWaypointMarkers = [];

  // Add numbered markers at each waypoint with distance labels
  dronePathPoints.forEach((point, index) => {
    // Calculate distance from this waypoint to the original route
    const closestPointInfo = findClosestPointOnRoute(point, routeCoordinates);
    const distanceFromRoute = closestPointInfo.distance * 1000; // Convert to meters

    // Create a custom div icon with the waypoint number
    const waypointIcon = L.divIcon({
      className: 'drone-waypoint-marker',
      html: `<div class="drone-waypoint-marker">${index + 1}</div>`,
      iconSize: [24, 24],
      iconAnchor: [12, 12]
    });

    // Add marker with waypoint number
    const marker = L.marker([point[0], point[1]], { icon: waypointIcon }).addTo(map);
    marker.bindPopup(`Waypoint ${index + 1}<br>Distance from route: ${distanceFromRoute.toFixed(1)} m`);
    state.droneWaypointMarkers.push(marker);

    // Add a label showing the distance from route
    const label = L.marker([point[0], point[1]], {
      icon: L.divIcon({
        className: 'distance-label',
        html: `${distanceFromRoute.toFixed(0)}m`,
        iconSize: [null, null],
        iconAnchor: [15, -15]
      }),
      interactive: false
    }).addTo(map);
    state.droneWaypointMarkers.push(label);

    // Draw a line from the waypoint to the closest point on the route
    const routeConnector = L.polyline([
      [point[0], point[1]],
      [closestPointInfo.point[0], closestPointInfo.point[1]]
    ], {
      color: '#FF4500',
      weight: 1.5,
      opacity: 0.6,
      dashArray: '3,5'
    }).addTo(map);
    state.droneWaypointMarkers.push(routeConnector);
  });
}

// 13. Execute path mission
function executePathMission() {
  const state = window.pathPredictorState;

  if (!state.droneWaypoints || state.droneWaypoints.length < 2) {
    addImportantMessage("No valid route to execute. Please calculate a route first.");
    return;
  }

  // Confirm with user
  if (!confirm("Are you sure you want to execute this convoy mission? The drone will follow the optimized path.")) {
    return;
  }

  // Convert drone waypoints to the format expected by the backend
  const missionWaypoints = state.droneWaypoints.map((point, index) => ({
    lat: point[0],
    lon: point[1],
    alt: 10, // Default altitude of 10 meters
    type: 'waypoint',
    name: `Waypoint ${index + 1}`
  }));

  // Show loading indicator
  showLoading();

  // Send request to the backend
  fetch('/upload_convoy_mission', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      waypoints: missionWaypoints
    })
  })
  .then(response => response.json())
  .then(data => {
    hideLoading();

    if (data.error) {
      throw new Error(data.error);
    }

    addImportantMessage(`Mission uploaded successfully with ${missionWaypoints.length} waypoints`);

    // Ask user if they want to execute the mission immediately
    if (confirm("Mission uploaded. Do you want to execute it now?")) {
      executeMission();
    }

    // Close the popup
    closePathPredictorPopup();
  })
  .catch(error => {
    hideLoading();
    console.error('Error uploading mission:', error);
    addImportantMessage("Failed to upload mission: " + error.message);
  });
}

// 14. Clear route visuals
function clearConvoyRoute() {
  clearRouteVisuals();

  // Reset info text
  document.getElementById('path-distance').textContent = "-";
  document.getElementById('drone-path-distance').textContent = "-";
  document.getElementById('path-time').textContent = "-";
  document.getElementById('path-road-type').textContent = "-";
  document.getElementById('path-waypoints').textContent = "-";

  // Disable execute mission button
  document.getElementById('execute-path-mission-btn').disabled = true;

  addImportantMessage("Route cleared. You can calculate a new route.");
}

function clearRouteVisuals() {
  const state = window.pathPredictorState;

  // Clear route polyline
  if (state.routePolyline) {
    map.removeLayer(state.routePolyline);
    state.routePolyline = null;
  }

  // Clear drone path polyline
  if (state.dronePathPolyline) {
    map.removeLayer(state.dronePathPolyline);
    state.dronePathPolyline = null;
  }

  // Clear waypoint markers
  state.droneWaypointMarkers.forEach(marker => map.removeLayer(marker));
  state.droneWaypointMarkers = [];

  // Clear curve highlights
  if (window.curveHighlightPolylines) {
    window.curveHighlightPolylines.forEach(polyline => map.removeLayer(polyline));
    window.curveHighlightPolylines = [];
  }
}

// 15. Add OSRM-related utility functions
// Decode Google polyline format
function decodePolyline(encodedPolyline) {
  const points = [];
  let index = 0, lat = 0, lng = 0;

  while (index < encodedPolyline.length) {
    let b, shift = 0, result = 0;

    do {
      b = encodedPolyline.charCodeAt(index++) - 63;
      result |= (b & 0x1f) << shift;
      shift += 5;
    } while (b >= 0x20);

    const dlat = ((result & 1) ? ~(result >> 1) : (result >> 1));
    lat += dlat;

    shift = 0;
    result = 0;

    do {
      b = encodedPolyline.charCodeAt(index++) - 63;
      result |= (b & 0x1f) << shift;
      shift += 5;
    } while (b >= 0x20);

    const dlng = ((result & 1) ? ~(result >> 1) : (result >> 1));
    lng += dlng;

    points.push([lat / 1e5, lng / 1e5]);
  }

  return points;
}

// Function to calculate distance between two points (Haversine formula)
function calculateHaversineDistance(lat1, lon1, lat2, lon2) {
  const R = 6371; // Radius of the earth in km
  const dLat = deg2rad(lat2 - lat1);
  const dLon = deg2rad(lon2 - lon1);
  const a =
    Math.sin(dLat/2) * Math.sin(dLat/2) +
    Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) *
    Math.sin(dLon/2) * Math.sin(dLon/2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  const distance = R * c; // Distance in km
  return distance;
}

function deg2rad(deg) {
  return deg * (Math.PI/180);
}

function rad2deg(rad) {
  return rad * (180/Math.PI);
}

// Find the closest point on the route to a given point
function findClosestPointOnRoute(point, routeCoordinates) {
  let minDistance = Infinity;
  let closestPoint = null;
  let closestSegmentIndex = -1;

  // Check each segment of the route
  for (let i = 0; i < routeCoordinates.length - 1; i++) {
    const segmentStart = routeCoordinates[i];
    const segmentEnd = routeCoordinates[i + 1];

    // Calculate the closest point on this segment to our target point
    const closestPointOnSegment = findClosestPointOnSegment(
      point, segmentStart, segmentEnd
    );

    // Calculate distance to this point
    const distance = calculateHaversineDistance(
      point[0], point[1],
      closestPointOnSegment[0], closestPointOnSegment[1]
    );

    // If this is the closest so far, update our result
    if (distance < minDistance) {
      minDistance = distance;
      closestPoint = closestPointOnSegment;
      closestSegmentIndex = i;
    }
  }

  return {
    point: closestPoint,
    distance: minDistance,
    segmentIndex: closestSegmentIndex
  };
}

// Find the closest point on a line segment to a given point
function findClosestPointOnSegment(point, segmentStart, segmentEnd) {
  // Convert to Cartesian coords for vector math (rough approximation)
  const p = [point[1], point[0]]; // [lon, lat]
  const v = [segmentStart[1], segmentStart[0]];
  const w = [segmentEnd[1], segmentEnd[0]];

  // Calculate vectors
  const vw = [w[0] - v[0], w[1] - v[1]];
  const vp = [p[0] - v[0], p[1] - v[1]];

  // Calculate dot products
  const vwDotVw = vw[0] * vw[0] + vw[1] * vw[1];
  const vwDotVp = vw[0] * vp[0] + vw[1] * vp[1];

  // Calculate projection ratio
  let t = vwDotVp / vwDotVw;

  // Constrain to segment
  t = Math.max(0, Math.min(1, t));

  // Calculate closest point
  const closestPoint = [
    v[1] + t * (w[1] - v[1]), // lat
    v[0] + t * (w[0] - v[0])  // lon
  ];

  return closestPoint;
}

// Function to analyze if a route is straight or curved
function analyzeRouteCurvature(coordinates) {
  if (coordinates.length < 3) {
    return { description: "Too few points to analyze", curveSegments: [] };
  }

  // Calculate total route distance and direct distance
  let totalDistance = 0;
  let directDistance = calculateHaversineDistance(
    coordinates[0][0], coordinates[0][1],
    coordinates[coordinates.length-1][0], coordinates[coordinates.length-1][1]
  );

  // Calculate point-to-point distances
  for (let i = 0; i < coordinates.length - 1; i++) {
    const dist = calculateHaversineDistance(
      coordinates[i][0], coordinates[i][1],
      coordinates[i+1][0], coordinates[i+1][1]
    );
    totalDistance += dist;
  }

  // Calculate bearing changes to detect curves
  let bearingChanges = [];
  for (let i = 0; i < coordinates.length - 2; i++) {
    const bearing1 = calculateBearing(
      coordinates[i][0], coordinates[i][1],
      coordinates[i+1][0], coordinates[i+1][1]
    );
    const bearing2 = calculateBearing(
      coordinates[i+1][0], coordinates[i+1][1],
      coordinates[i+2][0], coordinates[i+2][1]
    );

    // Calculate the absolute bearing change
    let change = Math.abs(bearing2 - bearing1);
    if (change > 180) change = 360 - change;

    bearingChanges.push({
      index: i + 1,
      change: change
    });
  }

  // Identify curves based on bearing changes
  const CURVE_THRESHOLD = 15; // Angle change in degrees to consider a significant curve

  // Find segments with significant curves
  let curveSegments = [];
  let currentSegment = null;

  for (let i = 0; i < bearingChanges.length; i++) {
    const bc = bearingChanges[i];

    if (bc.change > CURVE_THRESHOLD) {
      // Start a new segment if we're not in one
      if (!currentSegment) {
        currentSegment = {
          startIndex: Math.max(0, bc.index - 1),
          endIndex: bc.index + 1,
          maxChange: bc.change
        };
      } else {
        // Extend current segment
        currentSegment.endIndex = bc.index + 1;
        currentSegment.maxChange = Math.max(currentSegment.maxChange, bc.change);
      }
    } else if (currentSegment) {
      // End the current segment
      curveSegments.push(currentSegment);
      currentSegment = null;
    }
  }

  // Add the last segment if there is one
  if (currentSegment) {
    curveSegments.push(currentSegment);
  }

  // Create curve segments with coordinates
  curveSegments = curveSegments.map(segment => {
    return {
      ...segment,
      coordinates: coordinates.slice(segment.startIndex, segment.endIndex + 1)
    };
  });

  // Merge segments that are close to each other
  curveSegments = mergeCloseSegments(curveSegments, 3); // Merge segments within 3 points

  // Calculate metrics
  const directness = directDistance / totalDistance;
  const avgBearingChange = bearingChanges.reduce((a, b) => a + b.change, 0) / bearingChanges.length;
  const sharpTurns = bearingChanges.filter(change => change.change > 45).length;

  // Determine if route is straight or curved
  let routeType = "";

  if (directness > 0.95 && avgBearingChange < 5) {
    routeType = "Very straight road";
  } else if (directness > 0.85 && avgBearingChange < 15) {
    routeType = "Mostly straight with minor curves";
  } else if (directness > 0.7 && avgBearingChange < 25) {
    routeType = "Moderately curved";
  } else {
    routeType = "Winding/curved road";
  }

  // Add additional info about sharp turns
  if (sharpTurns > 0) {
    routeType += ` with ${sharpTurns} sharp turn${sharpTurns > 1 ? 's' : ''}`;
  }

  return { description: routeType, curveSegments: curveSegments };
}

// Merge curve segments that are close to each other
function mergeCloseSegments(segments, maxGap) {
  if (segments.length <= 1) return segments;

  const mergedSegments = [];
  let currentSegment = segments[0];

  for (let i = 1; i < segments.length; i++) {
    const nextSegment = segments[i];

    // Check if segments are close
    if (nextSegment.startIndex - currentSegment.endIndex <= maxGap) {
      // Merge segments
      currentSegment = {
        startIndex: currentSegment.startIndex,
        endIndex: nextSegment.endIndex,
        maxChange: Math.max(currentSegment.maxChange, nextSegment.maxChange),
        coordinates: [...currentSegment.coordinates, ...nextSegment.coordinates.slice(1)]
      };
    } else {
      // Add current segment and start a new one
      mergedSegments.push(currentSegment);
      currentSegment = nextSegment;
    }
  }

  // Add the last segment
  mergedSegments.push(currentSegment);

  return mergedSegments;
}

// Highlight curve segments on the map
function highlightCurveSegments(curveSegments) {
  // Initialize curve highlights array if it doesn't exist
  window.curveHighlightPolylines = window.curveHighlightPolylines || [];

  // Remove existing curve highlights
  window.curveHighlightPolylines.forEach(polyline => map.removeLayer(polyline));
  window.curveHighlightPolylines = [];

  // Add new curve highlights
  curveSegments.forEach(segment => {
    const polyline = L.polyline(segment.coordinates, {
      color: '#FF0000',
      weight: 8,
      opacity: 0.8
    }).addTo(map);

    // Add a popup with curve information
    polyline.bindPopup(`Curve with ${segment.maxChange.toFixed(1)}° angle change`);

    window.curveHighlightPolylines.push(polyline);
  });
}

// Function to calculate bearing between two points
function calculateBearing(lat1, lon1, lat2, lon2) {
  lat1 = deg2rad(lat1);
  lon1 = deg2rad(lon1);
  lat2 = deg2rad(lat2);
  lon2 = deg2rad(lon2);

  const y = Math.sin(lon2 - lon1) * Math.cos(lat2);
  const x = Math.cos(lat1) * Math.sin(lat2) -
            Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
  let bearing = Math.atan2(y, x);

  bearing = rad2deg(bearing);
  bearing = (bearing + 360) % 360; // Normalize to 0-360

  return bearing;
}

// Calculate Drone Path
function calculateDronePath(routeCoordinates, minWaypointDistance = 200) {
  // Step 1: Apply Douglas-Peucker algorithm for initial simplification
  const epsilon = 0.1; // Simplification threshold in meters (lower = more waypoints)
  let simplifiedPoints = douglasPeuckerSimplify(routeCoordinates, epsilon);

  // Step 2: Ensure waypoints are at least minWaypointDistance apart
  // and handle visibility constraints
  let finalPoints = [simplifiedPoints[0]]; // Start with the first point

  // Process each point after the first one
  for (let i = 1; i < simplifiedPoints.length; i++) {
    const lastAddedPoint = finalPoints[finalPoints.length - 1];
    const currentPoint = simplifiedPoints[i];

    // Calculate distance between the last added point and current point
    const distance = calculateHaversineDistance(
      lastAddedPoint[0], lastAddedPoint[1],
      currentPoint[0], currentPoint[1]
    ) * 1000; // Convert km to meters

    // If the distance is at least the minimum required, add the point
    if (distance >= minWaypointDistance) {
      finalPoints.push(currentPoint);
    } else if (i === simplifiedPoints.length - 1) {
      // Always include the last point regardless of distance
      finalPoints.push(currentPoint);
    }
    // Skip points that are too close
  }

  // Step 3: Add intermediate points for segments that are too far apart
  let distanceAdjustedPoints = [];
  for (let i = 0; i < finalPoints.length - 1; i++) {
    const startPoint = finalPoints[i];
    const endPoint = finalPoints[i + 1];
    distanceAdjustedPoints.push(startPoint);

    // Calculate distance between consecutive points
    const distance = calculateHaversineDistance(
      startPoint[0], startPoint[1],
      endPoint[0], endPoint[1]
    ) * 1000; // Convert km to meters

    // If the distance is too large, add intermediate points
    const maxDistance = minWaypointDistance * 1.5; // Allow up to 1.5x the minimum distance
    if (distance > maxDistance) {
      // Calculate how many intermediate points to add
      const numIntermediatePoints = Math.floor(distance / minWaypointDistance) - 1;

      for (let j = 1; j <= numIntermediatePoints; j++) {
        const fraction = j / (numIntermediatePoints + 1);
        const intermediatePoint = [
          startPoint[0] + fraction * (endPoint[0] - startPoint[0]),
          startPoint[1] + fraction * (endPoint[1] - startPoint[1])
        ];
        distanceAdjustedPoints.push(intermediatePoint);
      }
    }
  }

  // Add the final point
  distanceAdjustedPoints.push(finalPoints[finalPoints.length - 1]);

  // Step 4: Check for sharp curves and add waypoints if necessary
  const sharpCurveThreshold = 20; // Angle in degrees
  let curveAdjustedPoints = [];

  for (let i = 0; i < distanceAdjustedPoints.length - 2; i++) {
    curveAdjustedPoints.push(distanceAdjustedPoints[i]);

    const bearing1 = calculateBearing(
      distanceAdjustedPoints[i][0], distanceAdjustedPoints[i][1],
      distanceAdjustedPoints[i+1][0], distanceAdjustedPoints[i+1][1]
    );

    const bearing2 = calculateBearing(
      distanceAdjustedPoints[i+1][0], distanceAdjustedPoints[i+1][1],
      distanceAdjustedPoints[i+2][0], distanceAdjustedPoints[i+2][1]
    );

    // Calculate the absolute bearing change
    let change = Math.abs(bearing2 - bearing1);
    if (change > 180) change = 360 - change;

    // Add waypoint at significant bearing changes
    if (change > sharpCurveThreshold) {
      const curvePoint = distanceAdjustedPoints[i+1];

      // Check if this curve point is too close to the previous point
      const prevPoint = curveAdjustedPoints[curveAdjustedPoints.length - 1];
      const distToPrev = calculateHaversineDistance(
        prevPoint[0], prevPoint[1],
        curvePoint[0], curvePoint[1]
      ) * 1000; // Convert km to meters

      // Only add the curve point if it's far enough from the previous point
      if (distToPrev >= minWaypointDistance * 0.3) { // Allow 30% of the minimum distance for curve points
        curveAdjustedPoints.push(curvePoint);
      }
    }
  }

  // Add the last two points
  if (distanceAdjustedPoints.length >= 2) {
    const lastPoint = distanceAdjustedPoints[distanceAdjustedPoints.length - 1];
    const secondLastPoint = distanceAdjustedPoints[distanceAdjustedPoints.length - 2];

    // Check if the second last point is already included
    const lastIncludedPoint = curveAdjustedPoints[curveAdjustedPoints.length - 1];
    if (!arePointsEqual(lastIncludedPoint, secondLastPoint)) {
      curveAdjustedPoints.push(secondLastPoint);
    }

    // Check if the last point is already included
    if (!arePointsEqual(curveAdjustedPoints[curveAdjustedPoints.length - 1], lastPoint)) {
      curveAdjustedPoints.push(lastPoint);
    }
  }

  // Add waypoint numbering for UI display
  curveAdjustedPoints.forEach((point, index) => {
    point.waypointNumber = index + 1;
  });

  return curveAdjustedPoints;
}

// Douglas-Peucker algorithm for path simplification
function douglasPeuckerSimplify(points, epsilon) {
  if (points.length <= 2) {
    return points;
  }

  // Find the point with the maximum distance
  let maxDistance = 0;
  let maxDistanceIndex = 0;

  for (let i = 1; i < points.length - 1; i++) {
    const distance = perpendicularDistance(
      points[i],
      points[0],
      points[points.length - 1]
    );

    if (distance > maxDistance) {
      maxDistance = distance;
      maxDistanceIndex = i;
    }
  }

  // If max distance is greater than epsilon, recursively simplify
  if (maxDistance > epsilon) {
    // Recursive call
    const firstLine = douglasPeuckerSimplify(
      points.slice(0, maxDistanceIndex + 1),
      epsilon
    );
    const secondLine = douglasPeuckerSimplify(
      points.slice(maxDistanceIndex),
      epsilon
    );

    // Concat the two simplified segments
    return firstLine.slice(0, -1).concat(secondLine);
  } else {
    // Return the end points of the line
    return [points[0], points[points.length - 1]];
  }
}

// Calculate perpendicular distance from point to line
function perpendicularDistance(point, lineStart, lineEnd) {
  // Convert lat/lng to meters to get proper distance
  const x0 = point[1];      // longitude
  const y0 = point[0];      // latitude
  const x1 = lineStart[1];  // longitude
  const y1 = lineStart[0];  // latitude
  const x2 = lineEnd[1];    // longitude
  const y2 = lineEnd[0];    // latitude

  // Calculate the denominator of the formula
  const denominator = Math.sqrt(
    Math.pow(y2 - y1, 2) + Math.pow(x2 - x1, 2)
  );

  // Avoid division by zero
  if (denominator === 0) {
    return 0;
  }

  // Calculate perpendicular distance
  const numerator = Math.abs(
    (y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1
  );

  // Convert to kilometers
  const distanceInDegrees = numerator / denominator;
  return distanceInDegrees * 111.32; // Rough conversion to km at equator
}

// Check if two points are equal
function arePointsEqual(point1, point2) {
  return point1[0] === point2[0] && point1[1] === point2[1];
}

// Initialize the Path Predictor Convoy Follower on page load
document.addEventListener('DOMContentLoaded', function() {
  // Add styles
  addPathPredictorStyles();

  // Add button to the GPS controls section
  addPathPredictorButton();

  // Create the popup
  createPathPredictorPopup();

  // Initialize the Path Predictor
  initPathPredictor();

  console.log('Path Predictor Convoy Follower initialized');
});