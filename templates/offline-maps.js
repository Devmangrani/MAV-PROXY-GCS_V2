// offline-maps.js - Standalone offline maps functionality for Drone Command Center
// This script adds offline map capabilities to an existing Leaflet map implementation

(function() {
  // Wait for document to be ready before initializing
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initOfflineMaps);
  } else {
    // Document already loaded, initialize immediately
    initOfflineMaps();
  }

  // Reference to the main map
  let mainMap = null;
  let currentTileLayer = null;
  let isOfflineMode = false;

  // Tile providers
  const tileProviders = {
    arcgis: {
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      options: {
        attribution: 'Tiles &copy; Esri',
        maxZoom: 19
      }
    },
    osm: {
      url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
      options: {
        attribution: '&copy; OpenStreetMap contributors',
        maxZoom: 19
      }
    }
  };

  // Initialize offline maps functionality
  function initOfflineMaps() {
    console.log('Initializing offline maps functionality...');

    // Register Service Worker for tile caching
    if ('serviceWorker' in navigator) {
      navigator.serviceWorker.register('map-sw.js')
        .then(reg => console.log('Map service worker registered'))
        .catch(err => console.error('Map service worker registration failed:', err));
    }

    // Find map instance
    let checkMapInterval = setInterval(() => {
      if (window.map) {
        clearInterval(checkMapInterval);
        mainMap = window.map;

        // Find current tile layer
        mainMap.eachLayer(layer => {
          if (layer instanceof L.TileLayer) {
            currentTileLayer = layer;
          }
        });

        // Add offline controls to UI
        addOfflineMapControls();

        console.log('Offline maps initialized successfully');
      }
    }, 200);

    // Create service worker file if it doesn't exist
    createServiceWorkerFile();
  }

  // Create map service worker file
  function createServiceWorkerFile() {
    const swCode = `
      // map-sw.js - Service Worker for offline maps
      const CACHE_NAME = 'drone-gcs-map-tiles-v1';
      
      // Install event - create cache
      self.addEventListener('install', (event) => {
        console.log('Map Service Worker installed');
        self.skipWaiting();
      });
      
      // Activate event - clean up old caches
      self.addEventListener('activate', (event) => {
        console.log('Map Service Worker activated');
        event.waitUntil(
          caches.keys().then((cacheNames) => {
            return Promise.all(
              cacheNames.filter((cacheName) => {
                return cacheName.startsWith('drone-gcs-map-') && cacheName !== CACHE_NAME;
              }).map((cacheName) => {
                return caches.delete(cacheName);
              })
            );
          })
        );
      });
      
      // Check if a request is for a map tile
      function isMapTile(url) {
        return url.includes('/tile/') || 
               url.includes('.openstreetmap.org/') ||
               url.includes('arcgisonline.com/');
      }
      
      // Fetch event - serve cached tiles when offline
      self.addEventListener('fetch', (event) => {
        if (isMapTile(event.request.url)) {
          event.respondWith(
            caches.open(CACHE_NAME).then((cache) => {
              return cache.match(event.request).then((response) => {
                // Return cached response if available
                if (response) {
                  return response;
                }
                
                // Otherwise fetch from network and cache
                return fetch(event.request).then((networkResponse) => {
                  if (networkResponse.ok) {
                    cache.put(event.request, networkResponse.clone());
                  }
                  return networkResponse;
                }).catch((error) => {
                  console.error('Failed to fetch tile:', error);
                  return new Response('Map tile not available offline', { status: 404 });
                });
              });
            })
          );
        }
      });
    `;

    const blob = new Blob([swCode], { type: 'application/javascript' });
    const url = URL.createObjectURL(blob);

    // Download the service worker file
    const a = document.createElement('a');
    a.href = url;
    a.download = 'map-sw.js';
    a.style.display = 'none';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    // Add instructions to user
    console.log('Service worker file created. Please place "map-sw.js" in your website root directory.');
  }

  // Add offline map controls to UI
  function addOfflineMapControls() {
    // Find or create map controls section
    let mapControls = document.getElementById('map-controls');

    if (!mapControls) {
      // Try to find right panel
      const rightPanel = document.getElementById('right-panel');
      if (rightPanel) {
        mapControls = document.createElement('div');
        mapControls.id = 'map-controls';
        mapControls.innerHTML = '<h3>Map Controls</h3>';
        rightPanel.appendChild(mapControls);
      } else {
        console.error('Cannot find right panel to add offline map controls');
        return;
      }
    }

    // Create offline maps section
    const offlineSection = document.createElement('div');
    offlineSection.id = 'offline-maps-section';
    offlineSection.innerHTML = `
      <h3>Offline Maps</h3>
      <button class="button" id="download-area-btn">
        <i class="fas fa-download"></i> Download Current Area
      </button>
      <button class="button" id="download-mission-area-btn">
        <i class="fas fa-route"></i> Download Mission Area
      </button>
      <button class="button" id="toggle-offline-btn">
        <i class="fas fa-map"></i> Use Offline Maps
      </button>
      <button class="button" id="clear-cache-btn">
        <i class="fas fa-trash-alt"></i> Clear Map Cache
      </button>
      <div id="download-progress" style="display: none; margin-top: 10px;">
        <div style="color: white; font-size: 12px;">Downloading map tiles...</div>
        <div style="width: 100%; background-color: #333; margin-top: 5px;">
          <div id="download-progress-bar" style="height: 10px; width: 0%; background-color: #4CAF50;"></div>
        </div>
        <div id="download-status" style="color: white; font-size: 12px; margin-top: 5px;">0%</div>
      </div>
    `;

    // Add section to map controls
    mapControls.appendChild(offlineSection);

    // Add event listeners
    document.getElementById('download-area-btn').addEventListener('click', downloadVisibleArea);
    document.getElementById('download-mission-area-btn').addEventListener('click', downloadMissionArea);
    document.getElementById('toggle-offline-btn').addEventListener('click', toggleOfflineMode);
    document.getElementById('clear-cache-btn').addEventListener('click', clearMapCache);

    // Define functions in global scope so HTML onclick can use them
    window.downloadVisibleArea = downloadVisibleArea;
    window.downloadMissionArea = downloadMissionArea;
    window.toggleOfflineMode = toggleOfflineMode;
    window.clearMapCache = clearMapCache;

    console.log('Offline map controls added to UI');
  }

  // Download tiles for current visible area
  function downloadVisibleArea() {
    if (!mainMap) {
      console.error('Map not initialized');
      return;
    }

    // Get current bounds and zoom
    const bounds = mainMap.getBounds();
    const currentZoom = mainMap.getZoom();
    const minZoom = Math.max(currentZoom - 2, 10); // Don't go below zoom 10
    const maxZoom = Math.min(currentZoom + 1, 19); // Don't go above zoom 19

    // Download tiles
    downloadTiles(bounds, minZoom, maxZoom);
  }

  // Download tiles for mission area
  function downloadMissionArea() {
    if (!mainMap || !window.waypoints || window.waypoints.length < 2) {
      if (typeof addImportantMessage === 'function') {
        addImportantMessage('No mission waypoints found. Please create a mission first.');
      } else {
        alert('No mission waypoints found. Please create a mission first.');
      }
      return;
    }

    // Create bounds from waypoints
    const latLngs = window.waypoints.map(wp => L.latLng(wp.lat, wp.lon));
    const bounds = L.latLngBounds(latLngs);

    // Add buffer around bounds (20%)
    const center = bounds.getCenter();
    const north = bounds.getNorth();
    const south = bounds.getSouth();
    const east = bounds.getEast();
    const west = bounds.getWest();

    const latBuffer = (north - south) * 0.2;
    const lngBuffer = (east - west) * 0.2;

    const bufferedBounds = L.latLngBounds(
      L.latLng(south - latBuffer, west - lngBuffer),
      L.latLng(north + latBuffer, east + lngBuffer)
    );

    // Download tiles for mission area (zoom 12-18 provides good coverage)
    downloadTiles(bufferedBounds, 12, 18);
  }

  // Toggle between online and offline mode
  function toggleOfflineMode() {
    if (!mainMap || !currentTileLayer) {
      console.error('Map or tile layer not initialized');
      return;
    }

    isOfflineMode = !isOfflineMode;
    const toggleBtn = document.getElementById('toggle-offline-btn');

    // Remove current tile layer
    mainMap.removeLayer(currentTileLayer);

    if (isOfflineMode) {
      // Switch to OpenStreetMap (more likely to work offline)
      currentTileLayer = L.tileLayer(tileProviders.osm.url, tileProviders.osm.options);
      toggleBtn.innerHTML = '<i class="fas fa-globe"></i> Use Online Maps';

      if (typeof addImportantMessage === 'function') {
        addImportantMessage('Switched to offline map mode. Using cached tiles when available.');
      }
    } else {
      // Switch back to ArcGIS
      currentTileLayer = L.tileLayer(tileProviders.arcgis.url, tileProviders.arcgis.options);
      toggleBtn.innerHTML = '<i class="fas fa-map"></i> Use Offline Maps';

      if (typeof addImportantMessage === 'function') {
        addImportantMessage('Switched to online map mode.');
      }
    }

    // Add new tile layer to map
    currentTileLayer.addTo(mainMap);
  }

  // Clear map tile cache
  function clearMapCache() {
    if (!confirm('Are you sure you want to delete all cached map tiles?')) {
      return;
    }

    if ('caches' in window) {
      caches.keys().then(names => {
        for (let name of names) {
          if (name.includes('map-tiles')) {
            caches.delete(name);
          }
        }

        if (typeof addImportantMessage === 'function') {
          addImportantMessage('Map cache cleared successfully');
        } else {
          alert('Map cache cleared successfully');
        }
      });
    } else {
      if (typeof addImportantMessage === 'function') {
        addImportantMessage('Cache API not available in this browser');
      } else {
        alert('Cache API not available in this browser');
      }
    }
  }

  // Download and cache tiles for a given area
  function downloadTiles(bounds, minZoom, maxZoom) {
    if (!mainMap) return;

    // Show progress UI
    const progressDiv = document.getElementById('download-progress');
    const progressBar = document.getElementById('download-progress-bar');
    const progressStatus = document.getElementById('download-status');

    if (progressDiv) progressDiv.style.display = 'block';
    if (progressBar) progressBar.style.width = '0%';
    if (progressStatus) progressStatus.textContent = '0%';

    // Calculate tiles within bounds for each zoom level
    const tiles = [];
    for (let zoom = minZoom; zoom <= maxZoom; zoom++) {
      // Convert bounds to tile coordinates
      const northWest = mainMap.project(bounds.getNorthWest(), zoom).divideBy(256).floor();
      const southEast = mainMap.project(bounds.getSouthEast(), zoom).divideBy(256).ceil();

      const xMin = northWest.x;
      const xMax = southEast.x;
      const yMin = northWest.y;
      const yMax = southEast.y;

      // Add each tile to the list
      for (let x = xMin; x <= xMax; x++) {
        for (let y = yMin; y <= yMax; y++) {
          tiles.push({
            url: currentTileLayer._url.replace('{z}', zoom).replace('{x}', x).replace('{y}', y),
            zoom: zoom,
            x: x,
            y: y
          });
        }
      }
    }

    // Inform user about the download
    if (typeof addImportantMessage === 'function') {
      addImportantMessage(`Downloading ${tiles.length} map tiles for offline use...`);
    }

    // Download tiles
    let downloaded = 0;
    let successCount = 0;

    // Use caches API to store tiles
    caches.open('drone-gcs-map-tiles-v1').then(cache => {
      // Process tiles in batches to avoid overwhelming the browser
      const batchSize = 10;
      let batch = 0;

      function processBatch() {
        const startIdx = batch * batchSize;
        const endIdx = Math.min(startIdx + batchSize, tiles.length);

        if (startIdx >= tiles.length) {
          // All batches processed
          if (typeof addImportantMessage === 'function') {
            addImportantMessage(`Downloaded ${successCount} of ${tiles.length} map tiles for offline use.`);
          }

          // Hide progress after 3 seconds
          setTimeout(() => {
            if (progressDiv) progressDiv.style.display = 'none';
          }, 3000);

          return;
        }

        // Process current batch
        const promises = [];

        for (let i = startIdx; i < endIdx; i++) {
          const tile = tiles[i];

          // Fetch and cache tile
          const promise = fetch(tile.url)
            .then(response => {
              if (response.ok) {
                cache.put(tile.url, response.clone());
                successCount++;
              }
              downloaded++;

              // Update progress
              const percent = Math.round((downloaded / tiles.length) * 100);
              if (progressBar) progressBar.style.width = `${percent}%`;
              if (progressStatus) progressStatus.textContent = `${percent}% (${downloaded}/${tiles.length})`;

              return true;
            })
            .catch(error => {
              downloaded++;
              console.warn(`Failed to download tile: ${tile.url}`, error);

              // Update progress even on error
              const percent = Math.round((downloaded / tiles.length) * 100);
              if (progressBar) progressBar.style.width = `${percent}%`;
              if (progressStatus) progressStatus.textContent = `${percent}% (${downloaded}/${tiles.length})`;

              return false;
            });

          promises.push(promise);
        }

        // Process next batch once current batch is done
        Promise.all(promises).then(() => {
          batch++;
          setTimeout(processBatch, 100); // Add a small delay between batches
        });
      }

      // Start processing
      processBatch();
    });
  }
})();