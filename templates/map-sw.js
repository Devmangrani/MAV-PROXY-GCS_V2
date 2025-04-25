// Service Worker for offline map caching
const CACHE_NAME = 'drone-gcs-map-cache';

// Install event
self.addEventListener('install', event => {
  self.skipWaiting();
  console.log('Map Service Worker installed');
});

// Activate event
self.addEventListener('activate', event => {
  self.clients.claim();
  console.log('Map Service Worker activated');
});

// Check if a request is for a map tile
function isMapTile(url) {
  return url.includes('/tile/') ||
         url.includes('.openstreetmap.org/') ||
         url.includes('arcgisonline.com/');
}

// Fetch event - handle map tile requests
self.addEventListener('fetch', event => {
  const url = event.request.url;

  if (isMapTile(url)) {
    event.respondWith(
      caches.open(CACHE_NAME).then(cache => {
        // Try to get from cache first
        return cache.match(event.request).then(cachedResponse => {
          if (cachedResponse) {
            // Return cached tile
            return cachedResponse;
          }

          // Otherwise try to fetch from network
          return fetch(event.request)
            .then(networkResponse => {
              // Cache a copy of the response
              cache.put(event.request, networkResponse.clone());
              return networkResponse;
            })
            .catch(error => {
              console.error('Failed to fetch map tile:', error);
              return new Response('Tile not available', { status: 404 });
            });
        });
      })
    );
  }
});