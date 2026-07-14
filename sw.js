// 정글짐 설계 서비스워커 — 네트워크 우선, 오프라인 시 캐시 폴백
// 항상 최신을 보여주기 위해 네트워크 요청은 브라우저 HTTP 캐시를 우회(reload)하고,
// 새 버전이 뜨면 이전 캐시를 지운다.
const CACHE = 'junglegym-v2';

self.addEventListener('install', () => self.skipWaiting());
self.addEventListener('activate', (e) => e.waitUntil((async () => {
  const keys = await caches.keys();
  await Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)));
  await self.clients.claim();
})()));

self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET') return;
  if (new URL(req.url).origin !== self.location.origin) return;
  e.respondWith(
    fetch(req, { cache: 'reload' })   // 서버에서 항상 새로 받아옴(브라우저 캐시 무시)
      .then((res) => {
        const copy = res.clone();
        caches.open(CACHE).then((c) => c.put(req, copy)).catch(() => {});
        return res;
      })
      .catch(() => caches.match(req))   // 오프라인일 때만 캐시 사용
  );
});
