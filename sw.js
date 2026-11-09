// Cache per usare il visualizzatore anche senza connessione.
const VERSION = 'fview-v2';
const SHELL = ['./', './index.html', './manifest.webmanifest', './icon-192.png', './icon-512.png', './favicon.png'];
const THREE = 'https://cdn.jsdelivr.net/npm/three@0.170.0/';
const LIBS = ['build/three.module.js', 'examples/jsm/controls/OrbitControls.js', 'examples/jsm/loaders/STLLoader.js',
  'examples/jsm/loaders/OBJLoader.js', 'examples/jsm/loaders/GLTFLoader.js', 'examples/jsm/utils/BufferGeometryUtils.js'].map(p => THREE + p);

self.addEventListener('install', e => {
  e.waitUntil(caches.open(VERSION).then(async c => {
    await c.addAll(SHELL);
    await Promise.all(LIBS.map(u => c.add(u).catch(() => {})));
  }).then(() => self.skipWaiting()));
});
self.addEventListener('activate', e => {
  e.waitUntil(caches.keys().then(keys => Promise.all(keys.filter(k => k !== VERSION).map(k => caches.delete(k)))).then(() => self.clients.claim()));
});
self.addEventListener('fetch', e => {
  const req = e.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);
  // pagina: prima la rete (per avere gli aggiornamenti), poi la cache
  if (url.origin === location.origin && (req.mode === 'navigate' || url.pathname.endsWith('.html'))) {
    e.respondWith(fetch(req).then(r => { const cp = r.clone(); caches.open(VERSION).then(c => c.put(req, cp)); return r; })
      .catch(() => caches.match(req).then(r => r || caches.match('./index.html'))));
    return;
  }
  // librerie e icone: prima la cache
  if (url.origin === location.origin || req.url.startsWith(THREE)) {
    e.respondWith(caches.match(req).then(hit => hit || fetch(req).then(r => {
      if (r.ok) { const cp = r.clone(); caches.open(VERSION).then(c => c.put(req, cp)); }
      return r;
    })));
  }
});
