/* 정글짐 3D 설계 도구
 * - 노드 = 조인트(중심점), 엣지 = 파이프
 * - 파이프 길이 L 로 연결된 두 조인트의 중심간 거리 = L + JOINT_SPAN
 * - 조인트 종류는 연결된 파이프 개수(degree)로 판정
 */

// ===== 상수 =====
const PIPE_LENGTHS = [10, 15, 20, 35];       // cm
const JOINT_SPAN = 5;                        // 조인트 양쪽 파이프 삽입 시 노출 길이(cm)
const JOINT_RADIUS = JOINT_SPAN / 2;         // 2.5cm
const PIPE_RADIUS = 1.2;
const SNAP_TOL = 0.6;                         // 기존 조인트 스냅 허용 오차(cm)

// 6방향 (dir 벡터 + 축 이름)
const DIRS = [
  { name: '+X', v: new THREE.Vector3( 1, 0, 0), axis: 'x' },
  { name: '-X', v: new THREE.Vector3(-1, 0, 0), axis: 'x' },
  { name: '+Y', v: new THREE.Vector3( 0, 1, 0), axis: 'y' },
  { name: '-Y', v: new THREE.Vector3( 0,-1, 0), axis: 'y' },
  { name: '+Z', v: new THREE.Vector3( 0, 0, 1), axis: 'z' },
  { name: '-Z', v: new THREE.Vector3( 0, 0,-1), axis: 'z' },
];

// ===== 데이터 모델 =====
let joints = [];   // {id, x, y, z}
let pipes = [];    // {id, a, b, length, axis}
let panels = [];   // {id, c:[id0,id1,id2,id3]} — 사각형 순서대로의 조인트 4개
let bridges = [];  // {id, c:[id0,id1,id2,id3]} — 수평 면 위의 흔들다리
let notes = [];    // {id, x, y, z, text} — 설계 메모
let rulers = [];   // {id, ...3D 끝점, length} — 줄자
let groups = [];   // {id, m:[{type,id}]} — 오브젝트 그룹
let nextId = 1;
let activeLength = 20;
let cubeHeight = 20;   // 육면체 높이(cm) — 가로/세로와 별도로 선택
let selection = [];    // 다중 선택: [{type:'joint'|'pipe'|'panel'|'bridge', id}]
function isSelected(type, id) { return selection.some(s => s.type === type && s.id === id); }
function clearSel() { selection = []; }
// 그룹에 속하면 그룹 전체 멤버, 아니면 자기 자신
function groupMembersOf(type, id) {
  const g = groups.find(gr => gr.m.some(m => m.type === type && m.id === id));
  return g ? g.m.map(m => ({ type: m.type, id: m.id })) : [{ type, id }];
}
function selectOne(type, id) { selection = groupMembersOf(type, id); }
function toggleSel(type, id) {
  const mem = groupMembersOf(type, id);
  const on = mem.every(m => isSelected(m.type, m.id));
  if (on) selection = selection.filter(s => !mem.some(m => m.type === s.type && m.id === s.id));
  else for (const m of mem) if (!isSelected(m.type, m.id)) selection.push(m);
}
let mode = 'pipe';     // 'pipe' | 'cube'

// 육면체 옥탄트 8방향 (조인트를 한 꼭짓점으로 두고 뻗는 방향)
const OCTANTS = [];
for (const sx of [1, -1]) for (const sy of [1, -1]) for (const sz of [1, -1]) OCTANTS.push({ sx, sy, sz });

function uid() { return nextId++; }

function addJoint(x, y, z) {
  const j = { id: uid(), x, y, z };
  joints.push(j);
  return j;
}
function getJoint(id) { return joints.find(j => j.id === id); }
function findJointNear(x, y, z) {
  return joints.find(j =>
    Math.abs(j.x - x) < SNAP_TOL &&
    Math.abs(j.y - y) < SNAP_TOL &&
    Math.abs(j.z - z) < SNAP_TOL);
}
function pipeBetween(idA, idB) {
  return pipes.find(p => (p.a === idA && p.b === idB) || (p.a === idB && p.b === idA));
}
function jointDegreeDirs(jointId) {
  // 해당 조인트에 연결된 파이프들의 방향 벡터 목록
  const dirs = [];
  for (const p of pipes) {
    let other = null;
    if (p.a === jointId) other = getJoint(p.b);
    else if (p.b === jointId) other = getJoint(p.a);
    if (!other) continue;
    const self = getJoint(jointId);
    dirs.push(new THREE.Vector3(other.x - self.x, other.y - self.y, other.z - self.z).normalize());
  }
  return dirs;
}

// 파이프 4개가 이루는 축정렬 사각형(면)들을 찾는다.
// 반환: [{key, c:[J,A,K,B](정렬 아닌 사각형 순서 id), w, h}]
function findFaces() {
  const found = new Map();
  for (const J of joints) {
    const nb = [];
    for (const p of pipes) {
      if (p.a === J.id) nb.push({ j: getJoint(p.b), len: p.length });
      else if (p.b === J.id) nb.push({ j: getJoint(p.a), len: p.length });
    }
    for (let i = 0; i < nb.length; i++) for (let k = i + 1; k < nb.length; k++) {
      const A = nb[i].j, B = nb[k].j;
      if (!A || !B) continue;
      const va = { x: A.x - J.x, y: A.y - J.y, z: A.z - J.z };
      const vb = { x: B.x - J.x, y: B.y - J.y, z: B.z - J.z };
      if (va.x * vb.x + va.y * vb.y + va.z * vb.z !== 0) continue;  // 서로 다른 축(수직)
      const K = findJointNear(A.x + vb.x, A.y + vb.y, A.z + vb.z);   // 네번째 꼭짓점
      if (!K) continue;
      if (!pipeBetween(A.id, K.id) || !pipeBetween(B.id, K.id)) continue;
      const key = [J.id, A.id, B.id, K.id].sort((a, b) => a - b).join(',');
      if (found.has(key)) continue;
      found.set(key, { key, c: [J.id, A.id, K.id, B.id], w: nb[i].len, h: nb[k].len });
    }
  }
  return found;
}
function faceKey(cornerIds) { return cornerIds.slice().sort((a, b) => a - b).join(','); }
function panelValid(panel) {
  const c = panel.c.map(getJoint);
  if (c.some(x => !x)) return false;
  for (let i = 0; i < 4; i++) if (!pipeBetween(panel.c[i], panel.c[(i + 1) % 4])) return false;
  return true;
}
function panelExists(cornerIds) {
  const k = faceKey(cornerIds);
  return panels.some(p => faceKey(p.c) === k);
}
// 면의 두 변 길이(정렬)
function panelSize(panel) {
  const p1 = pipeBetween(panel.c[0], panel.c[1]);
  const p2 = pipeBetween(panel.c[1], panel.c[2]);
  const a = p1 ? p1.length : 0, b = p2 ? p2.length : 0;
  return [Math.min(a, b), Math.max(a, b)];
}
// 흔들다리: 4개 조인트(사각형)에 매달림
function bridgeExists(cornerIds) { const k = faceKey(cornerIds); return bridges.some(b => faceKey(b.c) === k); }
function bridgeValid(b) { return b.c.length === 4 && b.c.every(id => joints.some(j => j.id === id)); }
function bridgeSpan(b) {   // 두 변 길이(정렬, cm)
  const c = b.c.map(getJoint);
  if (c.some(x => !x)) return [0, 0];
  const d = (P, Q) => Math.round(Math.hypot(P.x - Q.x, P.y - Q.y, P.z - Q.z));
  const e1 = d(c[0], c[1]), e2 = d(c[1], c[2]);
  return [Math.min(e1, e2), Math.max(e1, e2)];
}
// 4개 조인트를 사각형 둘레 순서로 정렬 (중심 기준 각도)
function orderQuad(ids) {
  const P = ids.map(getJoint);
  const c = new THREE.Vector3(); P.forEach(p => c.add(new THREE.Vector3(p.x, p.y, p.z))); c.multiplyScalar(0.25);
  const p0 = new THREE.Vector3(P[0].x, P[0].y, P[0].z);
  let n = p0.clone().sub(c).cross(new THREE.Vector3(P[1].x, P[1].y, P[1].z).sub(c));
  if (n.length() < 0.01) n = new THREE.Vector3(0, 1, 0);
  n.normalize();
  const u = p0.clone().sub(c).normalize();
  const w = n.clone().cross(u);
  return ids.slice().sort((iA, iB) => {
    const A = getJoint(iA), B = getJoint(iB);
    const va = new THREE.Vector3(A.x, A.y, A.z).sub(c), vb = new THREE.Vector3(B.x, B.y, B.z).sub(c);
    return Math.atan2(va.dot(w), va.dot(u)) - Math.atan2(vb.dot(w), vb.dot(u));
  });
}

// ===== Three.js 셋업 =====
const viewport = document.getElementById('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14161b);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 5000);      // 3D 원근 카메라
camera.position.set(120, 110, 170);
const VIEW2D_SIZE = 130;                                          // 2D 정투영 반높이(cm)
const orthoCam = new THREE.OrthographicCamera(-VIEW2D_SIZE, VIEW2D_SIZE, VIEW2D_SIZE, -VIEW2D_SIZE, 0.1, 3000);
let activeCam = camera;                                           // 현재 렌더/레이캐스트에 쓰는 카메라

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
viewport.appendChild(renderer.domElement);

// 메모 HTML 오버레이 레이어
const noteLayer = document.createElement('div');
noteLayer.id = 'note-layer';
viewport.appendChild(noteLayer);

// 줄자 길이 라벨 오버레이 레이어
const rulerLayer = document.createElement('div');
rulerLayer.id = 'ruler-layer';
viewport.appendChild(rulerLayer);

// 선택 모드 드래그 박스 오버레이
const selectBox = document.createElement('div');
selectBox.id = 'select-box';
selectBox.style.display = 'none';
viewport.appendChild(selectBox);
let boxStart = null;   // {x,y} 클라이언트 좌표
function showSelectBox(x, y) { boxStart = { x, y }; updateSelectBox(x, y); selectBox.style.display = 'block'; }
function updateSelectBox(x, y) {
  if (!boxStart) return;
  const r = viewport.getBoundingClientRect();
  selectBox.style.left = (Math.min(boxStart.x, x) - r.left) + 'px';
  selectBox.style.top = (Math.min(boxStart.y, y) - r.top) + 'px';
  selectBox.style.width = Math.abs(x - boxStart.x) + 'px';
  selectBox.style.height = Math.abs(y - boxStart.y) + 'px';
}
function hideSelectBox() { selectBox.style.display = 'none'; boxStart = null; }

let controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.1;

let view = '3d';       // '3d' | '2d'
let plane = 'top';     // 2D 평면: 'top'(위·평면도, 기본) | 'front'(앞) | 'side'(옆)

function structureCenter() {
  if (joints.length === 0) return new THREE.Vector3(0, 12, 0);
  const min = { x: Infinity, y: Infinity, z: Infinity }, max = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const j of joints) for (const ax of ['x', 'y', 'z']) { min[ax] = Math.min(min[ax], j[ax]); max[ax] = Math.max(max[ax], j[ax]); }
  return new THREE.Vector3((min.x + max.x) / 2, (min.y + max.y) / 2, (min.z + max.z) / 2);
}

// 뷰(2D/3D) 및 평면 적용 — 컨트롤을 다시 만들어 카메라를 교체
function applyView() {
  const center = structureCenter();
  controls.dispose();
  if (view === '3d') {
    activeCam = camera;
    controls = new THREE.OrbitControls(activeCam, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.1;
    controls.target.copy(center);
  } else {
    activeCam = orthoCam;
    orthoCam.zoom = 1;
    const D = 800;
    if (plane === 'front') { orthoCam.position.set(center.x, center.y, center.z + D); orthoCam.up.set(0, 1, 0); }
    else if (plane === 'side') { orthoCam.position.set(center.x + D, center.y, center.z); orthoCam.up.set(0, 1, 0); }
    else { orthoCam.position.set(center.x, center.y + D, center.z); orthoCam.up.set(0, 0, -1); }
    orthoCam.lookAt(center);
    controls = new THREE.OrbitControls(activeCam, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.1;
    controls.enableRotate = false;
    controls.target.copy(center);
  }
  applyControlButtons();
  controls.update();
  resize();
}

// 마우스 버튼 매핑: 선택/줄자 모드는 좌드래그를 박스/선긋기에 쓰기 위해 카메라 회전 비활성
function applyControlButtons() {
  if (mode === 'select' || mode === 'ruler') {
    controls.mouseButtons = { LEFT: -1, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: view === '3d' ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN };
  } else if (view === '2d') {
    controls.mouseButtons = { LEFT: THREE.MOUSE.PAN, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };
  } else {
    controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };
  }
}

// 조명
scene.add(new THREE.AmbientLight(0xffffff, 0.75));
const dir1 = new THREE.DirectionalLight(0xffffff, 0.7);
dir1.position.set(100, 200, 120);
scene.add(dir1);
const dir2 = new THREE.DirectionalLight(0xffffff, 0.3);
dir2.position.set(-120, 60, -100);
scene.add(dir2);

// 바닥 그리드 (10cm 간격)
const grid = new THREE.GridHelper(600, 60, 0x3a404d, 0x2a2e37);
scene.add(grid);
const axes = new THREE.AxesHelper(30);
scene.add(axes);

// 부품을 담는 그룹
const partsGroup = new THREE.Group();
scene.add(partsGroup);

// 면 모드: 채울 수 있는 면 미리보기 그룹
const facePreviewGroup = new THREE.Group();
scene.add(facePreviewGroup);

// 고스트 배치(템플릿/육면체) 그룹
const ghostGroup = new THREE.Group();
scene.add(ghostGroup);
const GHOST_BOX = new THREE.BoxGeometry(1, 1, 1);
const GHOST_EDGE = new THREE.EdgesGeometry(GHOST_BOX);
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

// 줄자(바닥 초록선) 그룹
const rulerGroup = new THREE.Group();
scene.add(rulerGroup);
const RULER_Y = 0.6;                 // 바닥보다 살짝 위 (z-fighting 방지)
const RULER_COLOR = 0x22c55e;        // 초록
const rulerMat = new THREE.MeshBasicMaterial({ color: RULER_COLOR });
// 초록 막대 + 양끝 눈금 (3D 끝점 a,b). 가로(바닥)·세로(수직) 모두 지원
function makeRulerMesh(r) {
  const g = new THREE.Group();
  const a = new THREE.Vector3(r.ax, r.ay, r.az), b = new THREE.Vector3(r.bx, r.by, r.bz);
  const dir = b.clone().sub(a), len = dir.length();
  const mid = a.clone().add(b).multiplyScalar(0.5);
  const up = new THREE.Vector3(0, 1, 0);
  const along = dir.clone().normalize();
  const bar = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 0.7, len, 8), rulerMat);
  bar.position.copy(mid); bar.quaternion.setFromUnitVectors(up, along);
  g.add(bar);
  // 눈금 방향: 막대에 수직 (수직 막대면 X축, 아니면 바닥 평면상 수직)
  const perp = Math.abs(along.y) > 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(-along.z, 0, along.x);
  for (const end of [a, b]) {
    const tick = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 0.7, 6, 8), rulerMat);
    tick.position.copy(end); tick.quaternion.setFromUnitVectors(up, perp);
    g.add(tick);
  }
  return g;
}
function rebuildRulers() {
  while (rulerGroup.children.length) {
    const m = rulerGroup.children.pop();
    m.traverse?.(o => { o.geometry?.dispose?.(); });
  }
  for (const r of rulers) {
    const m = makeRulerMesh(r);
    m.traverse(o => { if (o.isMesh) o.userData = { type: 'ruler', id: r.id }; });
    rulerGroup.add(m);
  }
  renderRulerLabels();
}
// 줄자 길이 라벨 (HTML 오버레이)
function renderRulerLabels() {
  rulerLayer.innerHTML = '';
  for (const r of rulers) {
    const el = document.createElement('div');
    el.className = 'ruler-label';
    el.dataset.id = r.id;
    const span = document.createElement('span');
    span.textContent = `${r.length}cm`;
    const del = document.createElement('button');
    del.className = 'ruler-del'; del.textContent = '×'; del.title = '줄자 삭제';
    del.addEventListener('click', () => withUndo(() => { rulers = rulers.filter(x => x.id !== r.id); rebuildRulers(); }));
    el.appendChild(span); el.appendChild(del);
    rulerLayer.appendChild(el);
  }
  positionRulerLabels();
}
const _rpos = new THREE.Vector3();
function positionRulerLabels() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  for (const el of rulerLayer.children) {
    const r = rulers.find(x => x.id === Number(el.dataset.id));
    if (!r) continue;
    _rpos.set((r.ax + r.bx) / 2, (r.ay + r.by) / 2, (r.az + r.bz) / 2).project(activeCam);
    if (_rpos.z > 1) { el.style.display = 'none'; continue; }
    el.style.display = 'flex';
    el.style.left = ((_rpos.x * 0.5 + 0.5) * w) + 'px';
    el.style.top = ((-_rpos.y * 0.5 + 0.5) * h) + 'px';
  }
}
// 드래그 중 미리보기 선
let rulerPreview = null;
function showRulerPreview(r) {
  hideRulerPreview();
  rulerPreview = makeRulerMesh(r);
  rulerPreview.traverse(o => { if (o.material) o.material = new THREE.MeshBasicMaterial({ color: RULER_COLOR, transparent: true, opacity: 0.5 }); });
  scene.add(rulerPreview);
}
function hideRulerPreview() { if (rulerPreview) { scene.remove(rulerPreview); rulerPreview = null; } }

function clearGhost() {
  while (ghostGroup.children.length) {
    const m = ghostGroup.children.pop();
    m.geometry?.dispose?.(); m.material?.dispose?.();
  }
}
// 셀 목록을 (ox,oy,oz) 기준으로 반투명 상자 고스트로 표시 (sx=가로/세로 간격, sy=높이 간격)
function showGhost(cells, ox, oy, oz, sx, sy = sx) {
  clearGhost();
  for (const [i, j, k] of cells) {
    const cx = ox + i * sx + sx / 2, cy = oy + j * sy + sy / 2, cz = oz + k * sx + sx / 2;
    const box = new THREE.Mesh(GHOST_BOX, new THREE.MeshBasicMaterial({ color: 0x4aa3ff, transparent: true, opacity: 0.18, depthWrite: false }));
    box.scale.set(sx, sy, sx); box.position.set(cx, cy, cz);
    const edge = new THREE.LineSegments(GHOST_EDGE, new THREE.LineBasicMaterial({ color: 0x8fcfff }));
    box.add(edge);
    ghostGroup.add(box);
  }
  ghostGroup.visible = true;
}
// 커서 → 바닥(y=0) 교차점
const _grPt = new THREE.Vector3();
function groundPoint() {
  raycaster.setFromCamera(pointer, activeCam);
  return raycaster.ray.intersectPlane(groundPlane, _grPt) ? _grPt : null;
}
function snapGrid(v, s) { return Math.round(v / s) * s; }

// (ax,az) 격자 칸(한 변 sx)의 발자국이 이미 채워진 가장 높은 y → 그 위에 쌓음 (없으면 0=바닥)
function stackBaseY(ax, az, sx) {
  const corners = [[ax, az], [ax + sx, az], [ax, az + sx], [ax + sx, az + sx]];
  const ys = new Set([0]);
  for (const j of joints)
    if (corners.some(c => Math.abs(j.x - c[0]) < SNAP_TOL && Math.abs(j.z - c[1]) < SNAP_TOL)) ys.add(j.y);
  let best = 0;
  for (const y of ys)
    if (corners.every(c => findJointNear(c[0], y, c[1]))) best = Math.max(best, y);
  return best;
}
// 육면체 배치 대상: 커서가 부품 위면 그 열, 아니면 바닥 격자. 스택이 있으면 그 위로.
function cubeGhostTarget() {
  const sx = activeLength + JOINT_SPAN, sy = cubeHeight + JOINT_SPAN;
  const hits = intersect(partsGroup.children);
  const pt = hits.length ? hits[0].point : groundPoint();
  if (!pt) return null;
  const ax = snapGrid(pt.x, sx), az = snapGrid(pt.z, sx);
  return { ax, ay: stackBaseY(ax, az, sx), az, sx, sy };
}

// 4개 조인트(사각형 순서)로 판넬 쿼드 메쉬 생성 (중심 기준 scale 만큼 안쪽으로)
function quadMesh(cornerJoints, material, scale) {
  const c = cornerJoints.map(j => new THREE.Vector3(j.x, j.y, j.z));
  const center = c.reduce((a, v) => a.add(v.clone()), new THREE.Vector3()).multiplyScalar(0.25);
  const v = c.map(p => center.clone().add(p.clone().sub(center).multiplyScalar(scale)));
  const pos = new Float32Array([
    v[0].x, v[0].y, v[0].z, v[1].x, v[1].y, v[1].z, v[2].x, v[2].y, v[2].z,
    v[0].x, v[0].y, v[0].z, v[2].x, v[2].y, v[2].z, v[3].x, v[3].y, v[3].z,
  ]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.computeVertexNormals();
  return new THREE.Mesh(geo, material);
}

// 작은 화살표(샤프트 + 머리) 생성 — 로컬 +Y를 dirVec 방향으로 회전
const ARROW_UP = new THREE.Vector3(0, 1, 0);
const SHAFT_GEO = new THREE.CylinderGeometry(0.6, 0.6, 6, 10);
const HEAD_GEO = new THREE.ConeGeometry(2, 3.5, 14);
function makeArrow(dirVec, userData) {
  const g = new THREE.Group();
  const mat = new THREE.MeshStandardMaterial({ color: 0xffd23f, metalness: 0.1, roughness: 0.5 });
  const shaft = new THREE.Mesh(SHAFT_GEO, mat); shaft.position.y = 6;      // 3~9
  const head = new THREE.Mesh(HEAD_GEO, mat); head.position.y = 10.75;     // 9~12.5
  g.add(shaft, head);
  g.quaternion.setFromUnitVectors(ARROW_UP, dirVec.clone().normalize());
  shaft.userData = head.userData = userData;
  return g;
}

// 파이프 모드: 6방향 축 화살표
const handleGroup = new THREE.Group();
handleGroup.visible = false;
scene.add(handleGroup);
const handleArrows = [];
for (const d of DIRS) {
  const arrow = makeArrow(d.v, { type: 'handle', dir: d });
  arrow.userData = { dir: d };
  handleGroup.add(arrow);
  handleArrows.push(arrow);
}
// 육면체 모드: 8방향(옥탄트) 대각선 화살표
const cubeGroup = new THREE.Group();
cubeGroup.visible = false;
scene.add(cubeGroup);
const cubeArrows = [];
for (const oct of OCTANTS) {
  const dv = new THREE.Vector3(oct.sx, oct.sy, oct.sz);
  const arrow = makeArrow(dv, { type: 'cubehandle', oct });
  arrow.userData = { oct };
  cubeGroup.add(arrow);
  cubeArrows.push(arrow);
}

// 보이는 핸들의 자식 메쉬만 모아 레이캐스트 대상으로
const visibleHandleMeshes = () => handleArrows.filter(a => a.visible).flatMap(a => a.children);
const visibleCubeMeshes = () => cubeArrows.filter(a => a.visible).flatMap(a => a.children);

let activeJoint = null; // 화살표가 표시된 조인트

// ===== 렌더링 (데이터 → 메쉬) =====
const jointGeo = new THREE.SphereGeometry(JOINT_RADIUS, 20, 16);

function rebuild() {
  // 깨진 면(파이프/조인트 삭제로 사각형이 무너진 경우) 정리
  panels = panels.filter(panelValid);
  bridges = bridges.filter(bridgeValid);   // 흔들다리: 임의 크기 닫힌 사각형(변=파이프체인)
  pruneGroups();                           // 사라진 멤버 정리

  // 기존 메쉬 제거
  while (partsGroup.children.length) {
    const m = partsGroup.children.pop();
    m.geometry?.dispose?.();
    m.material?.dispose?.();
  }

  // 조인트
  for (const j of joints) {
    const isSel = isSelected('joint', j.id);
    const mat = new THREE.MeshStandardMaterial({
      color: isSel ? 0x4aa3ff : 0x222831,
      emissive: isSel ? 0x1a4a80 : 0x000000,
      metalness: 0.2, roughness: 0.6,
    });
    const mesh = new THREE.Mesh(jointGeo, mat);
    mesh.position.set(j.x, j.y, j.z);
    mesh.userData = { type: 'joint', id: j.id };
    partsGroup.add(mesh);
  }

  // 파이프
  for (const p of pipes) {
    const a = getJoint(p.a), b = getJoint(p.b);
    if (!a || !b) continue;
    const va = new THREE.Vector3(a.x, a.y, a.z);
    const vb = new THREE.Vector3(b.x, b.y, b.z);
    const mid = va.clone().add(vb).multiplyScalar(0.5);
    const isSel = isSelected('pipe', p.id);
    // 실제 파이프 길이만큼만 그린다 (조인트 반경만큼 양쪽 비움)
    const geo = new THREE.CylinderGeometry(PIPE_RADIUS, PIPE_RADIUS, p.length, 12);
    const mat = new THREE.MeshStandardMaterial({
      color: isSel ? 0x4aa3ff : lengthColor(p.length),
      emissive: isSel ? 0x1a4a80 : 0x000000,
      metalness: 0.1, roughness: 0.5,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.copy(mid);
    const up = new THREE.Vector3(0, 1, 0);
    const axisDir = vb.clone().sub(va).normalize();
    mesh.quaternion.setFromUnitVectors(up, axisDir);
    mesh.userData = { type: 'pipe', id: p.id };
    partsGroup.add(mesh);
  }

  // 판넬(면)
  for (const panel of panels) {
    const isSel = isSelected('panel', panel.id);
    const mat = new THREE.MeshStandardMaterial({
      color: isSel ? 0x4aa3ff : 0xf5df7a,
      emissive: isSel ? 0x1a4a80 : 0x000000,
      transparent: true, opacity: 0.55, side: THREE.DoubleSide,
      metalness: 0.05, roughness: 0.7,
    });
    const mesh = quadMesh(panel.c.map(getJoint), mat, 0.9);
    mesh.userData = { type: 'panel', id: panel.id };
    partsGroup.add(mesh);
  }

  // 흔들다리
  for (const bridge of bridges) {
    const isSel = isSelected('bridge', bridge.id);
    for (const m of bridgeMeshes(bridge, isSel)) {
      m.userData = { type: 'bridge', id: bridge.id };
      partsGroup.add(m);
    }
  }

  rebuildFacePreviews();
}

// 흔들다리 메쉬: 4꼭짓점(순서대로) 사각형 위에 늘어진 밧줄 2가닥 + 가로 발판
function bridgeMeshes(bridge, isSel) {
  const P = bridge.c.map(getJoint);
  if (P.some(x => !x)) return [];
  const p = P.map(j => new THREE.Vector3(j.x, j.y, j.z));
  // 두 변 중 긴 쪽을 스팬(밧줄 방향), 짧은 쪽을 폭
  const eA = p[1].clone().sub(p[0]), eB = p[3].clone().sub(p[0]);
  const spanAlongA = eA.length() >= eB.length();
  const S0 = p[0], S1 = spanAlongA ? p[1] : p[3];   // 스팬 시작/끝
  const Wv = (spanAlongA ? eB : eA);                 // 폭 벡터
  const spanVec = S1.clone().sub(S0), L = spanVec.length();
  if (L < 1) return [];
  const railColor = isSel ? 0x4aa3ff : 0xcbb98a, plankColor = isSel ? 0x4aa3ff : 0xb98a52;
  const railMat = new THREE.MeshStandardMaterial({ color: railColor, roughness: 0.8 });
  const plankMat = new THREE.MeshStandardMaterial({ color: plankColor, roughness: 0.8 });
  const N = 10, sag = Math.min(L * 0.18, 14);
  const railPts = (wf) => {   // 폭 방향 위치 wf(0/1), N등분, 아래로 처짐
    const pts = [];
    for (let n = 0; n <= N; n++) { const t = n / N; const pt = S0.clone().add(spanVec.clone().multiplyScalar(t)).add(Wv.clone().multiplyScalar(wf)); pt.y -= sag * 4 * t * (1 - t); pts.push(pt); }
    return pts;
  };
  const bar = (Q, R) => {
    const len = Q.distanceTo(R); if (len < 0.1) return null;
    const m = new THREE.Mesh(new THREE.CylinderGeometry(0.7, 0.7, len, 6), plankMat);
    m.position.copy(Q).add(R).multiplyScalar(0.5);
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), R.clone().sub(Q).normalize());
    return m;
  };
  const meshes = [];
  const r1 = railPts(0), r2 = railPts(1);
  for (const pts of [r1, r2]) meshes.push(new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), N, 0.7, 6), railMat));
  for (let n = 1; n < N; n++) meshes.push(bar(r1[n], r2[n]));   // 발판
  return meshes.filter(Boolean);
}

// 면 모드에서 채울 수 있는 면 미리보기 갱신
const PREVIEW_MAT_BASE = { color: 0xffd23f, transparent: true, opacity: 0.3, side: THREE.DoubleSide, depthWrite: false };
function rebuildFacePreviews() {
  while (facePreviewGroup.children.length) {
    const m = facePreviewGroup.children.pop();
    m.geometry?.dispose?.();
    m.material?.dispose?.();
  }
  if (mode !== 'face') return;   // 면(1칸 사각형) 미리보기만
  for (const face of findFaces().values()) {
    if (panelExists(face.c)) continue;
    const mat = new THREE.MeshBasicMaterial({ ...PREVIEW_MAT_BASE });
    const mesh = quadMesh(face.c.map(getJoint), mat, 0.88);
    mesh.userData = { type: 'facepreview', face };
    facePreviewGroup.add(mesh);
  }
}

function lengthColor(L) {
  return { 10: 0xf2c14e, 15: 0xe0e0e0, 20: 0xf2e2a0, 35: 0xd9d9d9 }[L] || 0xcccccc;
}

// ===== 편집: 방향으로 파이프 추가 =====
function addInDirection(joint, d) {
  const spacing = activeLength + JOINT_SPAN;
  const tx = joint.x + d.v.x * spacing;
  const ty = joint.y + d.v.y * spacing;
  const tz = joint.z + d.v.z * spacing;

  if (ty < -SNAP_TOL) return;   // 바닥(0) 아래로는 배치 금지
  let target = findJointNear(tx, ty, tz);
  if (!target) target = addJoint(tx, ty, tz);
  if (!pipeBetween(joint.id, target.id)) {
    pipes.push({ id: uid(), a: joint.id, b: target.id, length: activeLength, axis: d.axis });
  }
  rebuild();
  refreshHandles();
  updateBOM();
}

// 최소 꼭짓점(x0,y0,z0)에서 +방향으로 한 변 L짜리 큐브의 조인트/파이프 추가 (화면 갱신 없음)
// 가로·세로 = L, 높이 = H (기본 L). 파이프: x·z변은 L, y변은 H
function stampCubeAt(x0, y0, z0, L, H = L) {
  const sx = L + JOINT_SPAN, sy = H + JOINT_SPAN;   // sx = 가로/세로 간격, sy = 높이 간격
  if (y0 < -SNAP_TOL) return;   // 바닥 아래로는 금지
  const corner = [];
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) for (let k = 0; k < 2; k++) {
    const x = x0 + i * sx, y = y0 + j * sy, z = z0 + k * sx;
    corner[i * 4 + j * 2 + k] = findJointNear(x, y, z) || addJoint(x, y, z);
  }
  const edge = (aI, bI, axis, len) => {
    const a = corner[aI], b = corner[bI];
    if (!pipeBetween(a.id, b.id)) pipes.push({ id: uid(), a: a.id, b: b.id, length: len, axis });
  };
  // i(x)/j(y)/k(z) 인덱스: i*4+j*2+k — x·z변은 L, y변(높이)은 H
  for (let j = 0; j < 2; j++) for (let k = 0; k < 2; k++) edge(0 * 4 + j * 2 + k, 1 * 4 + j * 2 + k, 'x', L);
  for (let i = 0; i < 2; i++) for (let k = 0; k < 2; k++) edge(i * 4 + 0 * 2 + k, i * 4 + 1 * 2 + k, 'y', H);
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) edge(i * 4 + j * 2 + 0, i * 4 + j * 2 + 1, 'z', L);
}

// 육면체 한 개 찍기: joint를 한 꼭짓점으로 두고 oct 방향으로 뻗는 상자(가로/세로 L, 높이 H)
function stampCube(joint, oct, L, H) {
  const sx = L + JOINT_SPAN, sy = H + JOINT_SPAN;
  const y0 = joint.y + Math.min(0, oct.sy) * sy;
  if (y0 < -SNAP_TOL) return;   // 바닥 아래로는 금지
  stampCubeAt(joint.x + Math.min(0, oct.sx) * sx, y0, joint.z + Math.min(0, oct.sz) * sx, L, H);
  rebuild();
  refreshHandles();
  updateBOM();
}

// oct 방향 상자의 8꼭짓점이 모두 이미 존재하면 true (sx=가로/세로 간격, sy=높이 간격)
function octantFilled(joint, oct, sx, sy) {
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) for (let k = 0; k < 2; k++) {
    if (!findJointNear(joint.x + i * oct.sx * sx, joint.y + j * oct.sy * sy, joint.z + k * oct.sz * sx)) return false;
  }
  return true;
}

// 활성 조인트 기준 현재 모드에 맞는 핸들 표시
function refreshHandles() {
  handleGroup.visible = false;
  cubeGroup.visible = false;
  if (!activeJoint || !getJoint(activeJoint.id)) return;
  const j = getJoint(activeJoint.id);

  if (mode === 'pipe') {
    handleGroup.position.set(j.x, j.y, j.z);
    const used = jointDegreeDirs(j.id);
    const spacing = activeLength + JOINT_SPAN;
    let anyVisible = false;
    for (const arrow of handleArrows) {
      const dv = arrow.userData.dir.v;
      const occupied = used.some(u => u.dot(dv) > 0.9);
      const underground = j.y + dv.y * spacing < -SNAP_TOL;
      arrow.visible = !occupied && !underground;
      if (arrow.visible) anyVisible = true;
    }
    handleGroup.visible = anyVisible;
  } else if (mode === 'cube') {
    cubeGroup.position.set(j.x, j.y, j.z);
    const sx = activeLength + JOINT_SPAN, sy = cubeHeight + JOINT_SPAN;
    let anyVisible = false;
    for (const arrow of cubeArrows) {
      const oct = arrow.userData.oct;
      arrow.scale.setScalar(1);
      const filled = octantFilled(j, oct, sx, sy);
      const underground = j.y + Math.min(0, oct.sy) * sy < -SNAP_TOL;
      arrow.visible = !filled && !underground;
      if (arrow.visible) anyVisible = true;
    }
    cubeGroup.visible = anyVisible;
  }
}

// ===== 삭제 (선택된 모든 부품) =====
function deleteSelected() {
  if (selection.length === 0) return;
  const del = (type) => new Set(selection.filter(s => s.type === type).map(s => s.id));
  const delPipes = del('pipe'), delJoints = del('joint'), delPanels = del('panel'), delBridges = del('bridge');
  if (delJoints.size) {
    // 조인트 삭제 시 연결된 파이프도 함께 제거
    pipes = pipes.filter(p => !delJoints.has(p.a) && !delJoints.has(p.b));
    joints = joints.filter(j => !delJoints.has(j.id));
    if (activeJoint && delJoints.has(activeJoint.id)) { activeJoint = null; handleGroup.visible = false; cubeGroup.visible = false; }
  }
  if (delPipes.size) pipes = pipes.filter(p => !delPipes.has(p.id));
  if (delPanels.size) panels = panels.filter(p => !delPanels.has(p.id));
  if (delBridges.size) bridges = bridges.filter(b => !delBridges.has(b.id));
  pruneOrphanJoints();   // 파이프가 하나도 안 붙은 조인트는 함께 제거 (둥둥 뜬 조인트 방지)
  clearSel();
  rebuild();
  updateBOM();
}
// 연결된 파이프가 없는 조인트 제거
function pruneOrphanJoints() {
  const used = new Set();
  for (const p of pipes) { used.add(p.a); used.add(p.b); }
  joints = joints.filter(j => used.has(j.id));
}

// 현재 선택에 포함된 모든 조인트 id (파이프/판넬/흔들다리의 꼭짓점 포함)
function selectedJointIds() {
  const s = new Set();
  for (const sel of selection) {
    if (sel.type === 'joint') s.add(sel.id);
    else if (sel.type === 'pipe') { const p = pipes.find(x => x.id === sel.id); if (p) { s.add(p.a); s.add(p.b); } }
    else if (sel.type === 'panel') { const q = panels.find(x => x.id === sel.id); if (q) q.c.forEach(id => s.add(id)); }
    else if (sel.type === 'bridge') { const q = bridges.find(x => x.id === sel.id); if (q) q.c.forEach(id => s.add(id)); }
  }
  return s;
}

// 같은 좌표의 조인트를 하나로 병합 (이동 후 겹친 부분 연결)
function mergeCoincidentJoints() {
  const byPos = new Map(), remap = new Map();
  for (const j of joints.slice().sort((a, b) => a.id - b.id)) {
    const key = `${Math.round(j.x)},${Math.round(j.y)},${Math.round(j.z)}`;
    if (byPos.has(key)) remap.set(j.id, byPos.get(key)); else byPos.set(key, j.id);
  }
  if (!remap.size) return;
  const R = id => remap.get(id) ?? id;
  joints = joints.filter(j => !remap.has(j.id));
  const seen = new Set(), np = [];
  for (const p of pipes) {
    const a = R(p.a), b = R(p.b); if (a === b) continue;
    const k = a < b ? `${a}-${b}` : `${b}-${a}`;
    if (seen.has(k)) continue; seen.add(k);
    p.a = a; p.b = b; np.push(p);
  }
  pipes = np;
  const fixQuad = arr => arr.map(x => { x.c = x.c.map(R); return x; }).filter(x => new Set(x.c).size === 4);
  panels = fixQuad(panels); bridges = fixQuad(bridges);
}

// ===== 그룹 =====
function memberExists(m) {
  return m.type === 'pipe' ? pipes.some(p => p.id === m.id)
    : m.type === 'joint' ? joints.some(j => j.id === m.id)
    : m.type === 'panel' ? panels.some(p => p.id === m.id)
    : m.type === 'bridge' ? bridges.some(b => b.id === m.id) : false;
}
function pruneGroups() {
  groups = groups.map(g => ({ ...g, m: g.m.filter(memberExists) })).filter(g => g.m.length >= 2);
}
function groupSelection() {
  if (selection.length < 2) { toast('2개 이상 선택해야 그룹으로 묶습니다'); return; }
  const k = s => `${s.type}:${s.id}`;
  const sel = new Set(selection.map(k));
  groups = groups.map(g => ({ ...g, m: g.m.filter(m => !sel.has(k(m))) })).filter(g => g.m.length >= 2);
  groups.push({ id: uid(), m: selection.map(s => ({ type: s.type, id: s.id })) });
  toast(`${selection.length}개를 그룹으로 묶었습니다`);
}
function ungroupSelection() {
  const k = s => `${s.type}:${s.id}`;
  const sel = new Set(selection.map(k));
  const before = groups.length;
  groups = groups.filter(g => !g.m.some(m => sel.has(k(m))));
  toast(groups.length < before ? '그룹을 해제했습니다' : '선택된 그룹이 없습니다');
}

let bridgePicks = [];   // 흔들다리: 클릭한 매달 조인트 id들 (4개 모이면 생성)

// ===== 그룹 장착(연결): 바닥 조인트 → 연결 조인트 → 대상 순으로 골라 기울여 붙임 =====
let mountPhase = -1;                 // -1 비활성, 0 바닥, 1 연결, 2 대상
let mountFloor = [], mountConn = [], mountTargets = [];   // Floor/Conn = 그룹 조인트 id, Targets = {x,y,z}
const mountMarkers = new THREE.Group();
scene.add(mountMarkers);

function mountReset() {
  mountPhase = mode === 'mount' ? 0 : -1;
  mountFloor = []; mountConn = []; mountTargets = [];
  renderMountMarkers(); updateMountPanel();
}
function renderMountMarkers() {
  while (mountMarkers.children.length) { const m = mountMarkers.children.pop(); m.geometry?.dispose?.(); m.material?.dispose?.(); }
  const put = (pos, color) => {
    const s = new THREE.Mesh(new THREE.SphereGeometry(3.4, 12, 10), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.85 }));
    s.position.set(pos.x, pos.y, pos.z); mountMarkers.add(s);
  };
  for (const id of mountFloor) { const j = getJoint(id); if (j) put(j, 0x4aa3ff); }   // 바닥=파랑
  for (const id of mountConn) { const j = getJoint(id); if (j) put(j, 0x22c55e); }     // 연결=초록
  for (const t of mountTargets) put(t, 0xff9f43);                                       // 대상=주황
}
function updateMountPanel() {
  const sec = document.getElementById('mount-section');
  if (!sec) return;
  const info = document.getElementById('mount-info');
  const btn = document.getElementById('btn-mount-next');
  const texts = [
    `① 바닥에 붙일 그룹 조인트 클릭 (${mountFloor.length}개)`,
    `② 구조에 연결할 그룹 조인트 클릭 (${mountConn.length}개)`,
    `③ 붙일 대상 조인트/파이프 클릭 (${mountTargets.length}개)`,
  ];
  info.textContent = mountPhase >= 0 ? texts[mountPhase] : '';
  btn.textContent = mountPhase < 2 ? '다음 단계 ▶' : '연결 실행 ✓';
}
function mountClick() {
  if (mountPhase < 0) mountPhase = 0;
  if (mountPhase === 2) {   // 대상: 조인트 우선, 없으면 파이프 중점
    const hj = intersect(partsGroup.children.filter(m => m.userData.type === 'joint'));
    if (hj.length) { const j = getJoint(hj[0].object.userData.id); mountTargets.push({ x: j.x, y: j.y, z: j.z }); }
    else { const hp = intersect(partsGroup.children.filter(m => m.userData.type === 'pipe')); if (hp.length) { const p = hp[0].point; mountTargets.push({ x: p.x, y: p.y, z: p.z }); } else return; }
  } else {                  // 바닥/연결: 그룹 조인트
    const hj = intersect(partsGroup.children.filter(m => m.userData.type === 'joint'));
    if (!hj.length) return;
    const id = hj[0].object.userData.id;
    const list = mountPhase === 0 ? mountFloor : mountConn;
    if (!list.includes(id)) list.push(id);
  }
  renderMountMarkers(); updateMountPanel();
}
function mountNext() {
  if (mountPhase < 0) { mountReset(); return; }
  if (mountPhase === 0 && !mountFloor.length) { toast('바닥에 붙일 조인트를 선택하세요'); return; }
  if (mountPhase === 1 && !mountConn.length) { toast('연결할 조인트를 선택하세요'); return; }
  if (mountPhase < 2) { mountPhase++; updateMountPanel(); return; }
  if (!mountTargets.length) { toast('붙일 대상을 선택하세요'); return; }
  withUndo(mountApply);
}
// 연결 조인트+바닥 조인트가 속한 연결요소 전체 (움직일 그룹)
function componentOf(seedIds) {
  const adj = new Map();
  for (const j of joints) adj.set(j.id, []);
  for (const p of pipes) { adj.get(p.a)?.push(p.b); adj.get(p.b)?.push(p.a); }
  const seen = new Set(seedIds), q = [...seedIds];
  while (q.length) { const id = q.shift(); for (const n of (adj.get(id) || [])) if (!seen.has(n)) { seen.add(n); q.push(n); } }
  return seen;
}
const v3 = (p) => new THREE.Vector3(p.x, p.y, p.z);
function centroidV(arr) { const c = new THREE.Vector3(); for (const p of arr) c.add(v3(p)); return arr.length ? c.multiplyScalar(1 / arr.length) : c; }
// 축 u(중심 O) 둘레로 각 join 회전
function rotateMovingAbout(movingSet, O, u, ang) {
  const cos = Math.cos(ang), sin = Math.sin(ang);
  for (const id of movingSet) {
    const j = getJoint(id); if (!j) continue;
    const p = new THREE.Vector3(j.x, j.y, j.z).sub(O);
    const along = u.clone().multiplyScalar(p.dot(u)), pp = p.clone().sub(along);
    const w = u.clone().cross(pp);
    const rp = pp.multiplyScalar(cos).add(w.multiplyScalar(sin));
    const np = O.clone().add(along).add(rp);
    j.x = np.x; j.y = np.y; j.z = np.z;
  }
}
function mountApply() {
  const movingSet = componentOf([...mountFloor, ...mountConn]);
  const move = (dx, dy, dz) => { for (const id of movingSet) { const j = getJoint(id); if (j) { j.x += dx; j.y += dy; j.z += dz; } } };
  const C = centroidV(mountConn.map(getJoint));   // 연결 조인트 중심(변환 전)
  const T = centroidV(mountTargets);              // 대상 중심
  // 1) 연결 중심을 대상으로 이동 (그룹 통째로 — 분리 없음)
  move(T.x - C.x, T.y - C.y, T.z - C.z);
  // 2) 대상(연결 중심)을 축으로 기울여 바닥 조인트 중심을 y=0 으로
  const F = centroidV(mountFloor.map(getJoint));
  const d = new THREE.Vector3(F.x - T.x, F.y - T.y, F.z - T.z);
  const hLen = Math.hypot(d.x, d.z);
  const R = Math.hypot(hLen, d.y);
  if (R > 0.01 && R >= Math.abs(T.y) - 0.5) {
    const hx = hLen > 0.01 ? d.x / hLen : 1, hz = hLen > 0.01 ? d.z / hLen : 0;
    const u = new THREE.Vector3(-hz, 0, hx);           // 수평 회전축 (d 수평성분에 수직)
    const a = hLen, b = d.y;                            // 회전면상 (ĥ, Y) 성분
    const phi = Math.atan2(b, a);
    const s = Math.max(-1, Math.min(1, (-T.y) / R));    // 회전 후 d.y = -T.y → F.y = 0
    const base = Math.asin(s);
    const c1 = base - phi, c2 = (Math.PI - base) - phi;
    const ang = Math.abs(((c2 + Math.PI) % (2 * Math.PI)) - Math.PI) < Math.abs(((c1 + Math.PI) % (2 * Math.PI)) - Math.PI) ? c2 : c1;
    rotateMovingAbout(movingSet, T, u, ang);
  }
  // 3) 연결 조인트를 가장 가까운 대상에 스냅(연결) 후 병합
  for (const id of mountConn) {
    const j = getJoint(id); if (!j) continue;
    let best = null, bd = Infinity;
    for (const tp of mountTargets) { const dd = v3(tp).distanceTo(v3(j)); if (dd < bd) { bd = dd; best = tp; } }
    if (best) { j.x = best.x; j.y = best.y; j.z = best.z; }
  }
  mergeCoincidentJoints();
  mountReset();
  rebuild(); updateBOM();
  toast('그룹을 연결했습니다');
}

// 선택된 파이프들의 길이를 newLen 으로 변경 후 전체 재배치
function setSelectedPipesLength(newLen) {
  const ids = new Set(selection.filter(s => s.type === 'pipe').map(s => s.id));
  if (!ids.size) return;
  for (const p of pipes) if (ids.has(p.id)) p.length = newLen;
  relayout();
  rebuild();
  updateBOM();
}

// 파이프 길이(간격)에 맞춰 조인트 위치를 다시 계산. 각 연결요소의 최저 조인트를 고정하고 BFS 전파.
function relayout() {
  const adj = new Map();
  for (const j of joints) adj.set(j.id, []);
  for (const p of pipes) { adj.get(p.a).push({ to: p.b, p }); adj.get(p.b).push({ to: p.a, p }); }

  const pos = new Map();
  const ordered = joints.slice().sort((a, b) => a.y - b.y || a.x - b.x || a.z - b.z);
  for (const start of ordered) {
    if (pos.has(start.id)) continue;
    pos.set(start.id, { x: start.x, y: start.y, z: start.z });   // 앵커 고정
    const queue = [start.id];
    while (queue.length) {
      const id = queue.shift();
      const P = pos.get(id), cur = getJoint(id);
      for (const { to, p } of adj.get(id)) {
        if (pos.has(to)) continue;
        const other = getJoint(to);
        const sign = Math.sign(other[p.axis] - cur[p.axis]) || 1;   // 기존 방향 유지
        const np = { x: P.x, y: P.y, z: P.z };
        np[p.axis] += (p.length + JOINT_SPAN) * sign;
        pos.set(to, np);
        queue.push(to);
      }
    }
  }
  for (const j of joints) { const np = pos.get(j.id); if (np) { j.x = np.x; j.y = np.y; j.z = np.z; } }
}

// ===== 레이캐스트 (호버 & 클릭) =====
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

function setPointer(e) {
  const r = renderer.domElement.getBoundingClientRect();
  pointer.x = ((e.clientX - r.left) / r.width) * 2 - 1;
  pointer.y = -((e.clientY - r.top) / r.height) * 2 + 1;
}
function intersect(objs) {
  raycaster.setFromCamera(pointer, activeCam);
  return raycaster.intersectObjects(objs, false);
}

// ===== 선택 모드: 클릭/시프트 선택 & 박스 선택 =====
function clickSelect(e) {
  setPointer(e);
  // 조인트가 커서 아래 있으면 우선 선택 (면/판넬보다 앞순위)
  const jointHit = intersect(partsGroup.children.filter(m => m.userData.type === 'joint'));
  const o = jointHit.length ? jointHit[0].object : (intersect(partsGroup.children)[0] || {}).object;
  if (o) {
    if (e.shiftKey) toggleSel(o.userData.type, o.userData.id);
    else selectOne(o.userData.type, o.userData.id);
  } else if (!e.shiftKey) {
    clearSel();
  }
  rebuild(); updateBOM();
}
// 드래그 박스 안에 든 파이프(중점 기준)를 선택
function boxSelect(x0, y0, x1, y1, additive) {
  const r = renderer.domElement.getBoundingClientRect();
  const minX = Math.min(x0, x1) - r.left, maxX = Math.max(x0, x1) - r.left;
  const minY = Math.min(y0, y1) - r.top, maxY = Math.max(y0, y1) - r.top;
  if (!additive) clearSel();
  const v = new THREE.Vector3();
  const inBox = (p) => {
    v.copy(p).project(activeCam);
    if (v.z > 1) return false;
    const sx = (v.x * 0.5 + 0.5) * r.width, sy = (-v.y * 0.5 + 0.5) * r.height;
    return sx >= minX && sx <= maxX && sy >= minY && sy <= maxY;
  };
  for (const p of pipes) {
    const a = getJoint(p.a), b = getJoint(p.b); if (!a || !b) continue;
    const mid = new THREE.Vector3((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2);
    if (inBox(mid) && !isSelected('pipe', p.id)) selection.push({ type: 'pipe', id: p.id });
  }
  rebuild(); updateBOM();
}

let down = null;   // {x,y}
let dragged = false;
let rulerStart = null;   // 줄자 드래그 시작점 {x,z}
let rulerDir = 'h';      // 줄자 방향: 'h'(가로 자유) | 'a'(직각/축맞춤) | 'v'(세로/수직)
// 직각 모드면 시작점 기준 가까운 축(X/Z)에 맞춰 끝점을 곧게
function rulerEnd(gp) {
  if (rulerDir === 'a') {
    if (Math.abs(gp.x - rulerStart.x) >= Math.abs(gp.z - rulerStart.z)) return { x: gp.x, z: rulerStart.z };
    return { x: rulerStart.x, z: gp.z };
  }
  return { x: gp.x, z: gp.z };
}
let moving = null;       // 이동 중: {kind, start:{x,z}, ...}

// ===== 실행 취소 / 다시 실행 (Ctrl+Z / Ctrl+Shift+Z) =====
let undoStack = [], redoStack = [], gestureBefore = null;
function snapState() { return JSON.stringify(currentDesign()); }
function commitUndo(before) {   // 변경이 있었으면 이전 상태를 기록
  if (before != null && before !== snapState()) { undoStack.push(before); if (undoStack.length > 60) undoStack.shift(); redoStack = []; }
}
function withUndo(fn) { const b = snapState(); fn(); commitUndo(b); }
function undo() {
  if (!undoStack.length) { toast('되돌릴 작업이 없습니다'); return; }
  redoStack.push(snapState());
  applyLoaded(JSON.parse(undoStack.pop()));
  toast('되돌리기');
}
function redo() {
  if (!redoStack.length) { toast('다시 실행할 작업이 없습니다'); return; }
  undoStack.push(snapState());
  applyLoaded(JSON.parse(redoStack.pop()));
  toast('다시 실행');
}

const intersectRulers = () => { raycaster.setFromCamera(pointer, activeCam); return raycaster.intersectObjects(rulerGroup.children, true); };

// 이동 중 드래그 적용 (바닥 격자 스냅)
function applyMove(e) {
  setPointer(e);
  const gp = groundPoint();
  if (!gp || !moving) return;
  if (moving.kind === 'parts') {
    const step = activeLength + JOINT_SPAN;
    const dx = Math.round((gp.x - moving.start.x) / step) * step;
    const dz = Math.round((gp.z - moving.start.z) / step) * step;
    for (const id of moving.ids) { const j = getJoint(id), s = moving.snap.get(id); if (j && s) { j.x = s.x + dx; j.z = s.z + dz; } }
    rebuild(); updateBOM();
  } else {   // ruler
    const dx = Math.round((gp.x - moving.start.x) / JOINT_SPAN) * JOINT_SPAN;
    const dz = Math.round((gp.z - moving.start.z) / JOINT_SPAN) * JOINT_SPAN;
    const s = moving.snap;
    moving.r.ax = s.ax + dx; moving.r.az = s.az + dz; moving.r.bx = s.bx + dx; moving.r.bz = s.bz + dz;
    moving.r.ay = s.ay; moving.r.by = s.by;   // 높이는 유지
    rebuildRulers();
  }
}

renderer.domElement.addEventListener('pointerdown', (e) => {
  down = { x: e.clientX, y: e.clientY };
  dragged = false;
  gestureBefore = snapState();   // 이 제스처 시작 전 상태 (undo용)
  if (mode === 'select' && e.button === 0) {
    setPointer(e);
    const gp = groundPoint();
    const partHit = intersect(partsGroup.children);
    const rulerHit = intersectRulers();
    // 선택된 부품 위에서 드래그 시작 → 부품 이동
    if (gp && partHit.length && isSelected(partHit[0].object.userData.type, partHit[0].object.userData.id)) {
      const ids = selectedJointIds();
      const snap = new Map();
      for (const id of ids) { const j = getJoint(id); snap.set(id, { x: j.x, y: j.y, z: j.z }); }
      moving = { kind: 'parts', start: { x: gp.x, z: gp.z }, ids, snap };
    } else if (gp && rulerHit.length) {   // 줄자 위 드래그 → 줄자 이동
      const r = rulers.find(x => x.id === rulerHit[0].object.userData.id);
      if (r) moving = { kind: 'ruler', start: { x: gp.x, z: gp.z }, r, snap: { ax: r.ax, ay: r.ay, az: r.az, bx: r.bx, by: r.by, bz: r.bz } };
    }
    if (!moving) showSelectBox(e.clientX, e.clientY);   // 그 외 → 박스 선택
  }
  if (mode === 'ruler' && e.button === 0) { setPointer(e); const gp = groundPoint(); rulerStart = gp ? { x: gp.x, z: gp.z } : null; }
});

renderer.domElement.addEventListener('pointermove', (e) => {
  if (down) {
    if (Math.abs(e.clientX - down.x) + Math.abs(e.clientY - down.y) > 5) dragged = true;
    if (mode === 'select' && boxStart) updateSelectBox(e.clientX, e.clientY);   // 박스 갱신
    if (mode === 'select' && moving) applyMove(e);                              // 부품/줄자 이동
    if (mode === 'ruler' && rulerStart && rulerDir !== 'v') { setPointer(e); const gp = groundPoint(); if (gp) { const e2 = rulerEnd(gp); showRulerPreview({ ax: rulerStart.x, ay: RULER_Y, az: rulerStart.z, bx: e2.x, by: RULER_Y, bz: e2.z }); } }
    return; // 드래그 중엔 호버 갱신 안 함
  }
  setPointer(e);
  if (mode === 'ruler') { renderer.domElement.style.cursor = 'crosshair'; return; }
  if (mode === 'select') {
    // 선택된 부품/줄자 위면 이동 커서, 그 외는 선택 커서
    const ph = intersect(partsGroup.children);
    const onSelected = ph.length && isSelected(ph[0].object.userData.type, ph[0].object.userData.id);
    renderer.domElement.style.cursor = (onSelected || intersectRulers().length) ? 'move' : (ph.length ? 'pointer' : 'crosshair');
    return;
  }
  // 배치(템플릿) 중: 고스트를 커서 따라 스냅
  if (placing) {
    updatePlacementGhost();
    renderer.domElement.style.cursor = placing._anchor ? 'copy' : 'default';
    return;
  }
  // 메모/흔들다리 모드: 부품(조인트) 위에서만 커서 pointer
  if (mode === 'memo' || mode === 'bridge') {
    renderer.domElement.style.cursor = intersect(partsGroup.children).length ? 'pointer' : 'default';
    return;
  }
  // 면 모드: 미리보기 강조
  if (mode === 'face') {
    facePreviewGroup.children.forEach(m => m.material.opacity = PREVIEW_MAT_BASE.opacity);
    const targets = facePreviewGroup.children.concat(partsGroup.children.filter(m => m.userData.type === 'panel'));
    const hits = intersect(targets);
    if (hits.length) {
      if (hits[0].object.userData.type === 'facepreview') hits[0].object.material.opacity = 0.45;
      renderer.domElement.style.cursor = 'pointer';
      return;
    }
    renderer.domElement.style.cursor = intersect(partsGroup.children).length ? 'pointer' : 'default';
    return;
  }
  // 화살표가 떠 있으면 그 위 커서 표시 (육면체는 가리키는 화살표 강조)
  const grp = mode === 'pipe' ? handleGroup : cubeGroup;
  if (grp.visible) {
    if (mode === 'cube') {
      cubeArrows.forEach(a => a.scale.setScalar(1));
      const h = intersect(visibleCubeMeshes());
      if (h.length) { h[0].object.parent.scale.setScalar(1.4); renderer.domElement.style.cursor = 'pointer'; return; }
    } else {
      if (intersect(visibleHandleMeshes()).length) { renderer.domElement.style.cursor = 'pointer'; return; }
    }
  }
  // 조인트/파이프 위면 커서만 pointer (화살표 표시/숨김은 클릭에서 처리)
  const hit = intersect(partsGroup.children);
  // 육면체 모드: 스냅 고스트 (조인트 위에선 화살표 우선이라 숨김). 기존 큐브 위엔 얹힘.
  if (mode === 'cube') {
    const overJoint = hit.length && hit[0].object.userData.type === 'joint';
    const t = (!cubeGroup.visible && !overJoint) ? cubeGhostTarget() : null;
    if (t) {
      showGhost([[0, 0, 0]], t.ax, t.ay, t.az, t.sx, t.sy);
      renderer.domElement.style.cursor = 'copy';
      return;
    }
    clearGhost(); ghostGroup.visible = false;
  }
  renderer.domElement.style.cursor = hit.length ? 'pointer' : 'default';
});

renderer.domElement.addEventListener('pointerup', (e) => {
  handlePointerUp(e);
  commitUndo(gestureBefore);   // 이 제스처로 바뀐 게 있으면 undo 기록
  gestureBefore = null;
});
function handlePointerUp(e) {
  const wasDrag = dragged;
  const start = down;
  down = null; dragged = false;

  // 선택 모드: 이동 마무리 / 박스 선택 / 클릭 선택
  if (mode === 'select' && e.button === 0) {
    if (moving) {   // 이동 중이었으면 마무리 (부품 이동 시 겹친 조인트 병합)
      if (moving.kind === 'parts' && wasDrag) { mergeCoincidentJoints(); clearSel(); rebuild(); updateBOM(); }
      moving = null;
      return;
    }
    hideSelectBox();
    if (wasDrag && start) boxSelect(start.x, start.y, e.clientX, e.clientY, e.shiftKey);
    else clickSelect(e);
    return;
  }

  // 줄자 모드: 바닥에 그은 선 → 길이 입력받아 초록 줄자 생성
  if (mode === 'ruler' && e.button === 0) {
    hideRulerPreview();
    if (rulerStart && rulerDir === 'v') {
      // 세로(수직): 클릭한 바닥점에서 위로 입력 높이만큼
      const input = prompt('세로 줄자 높이(cm)', 100);
      const h = parseFloat(input);
      if (h > 0) {
        rulers.push({ id: uid(), ax: rulerStart.x, ay: RULER_Y, az: rulerStart.z, bx: rulerStart.x, by: RULER_Y + h, bz: rulerStart.z, length: Math.round(h) });
        rebuildRulers();
      }
    } else if (rulerStart && wasDrag) {
      // 가로(자유)/직각(축): 그은 방향(직각이면 축맞춤)으로 입력 길이만큼
      setPointer(e);
      const gp = groundPoint();
      if (gp) {
        const e2 = rulerEnd(gp);
        const dx = e2.x - rulerStart.x, dz = e2.z - rulerStart.z;
        const measured = Math.round(Math.hypot(dx, dz));
        if (measured > 1) {
          const input = prompt('줄자 길이(cm) — 비우면 실측값 사용', measured);
          if (input !== null) {
            let len = parseFloat(input);
            if (!(len > 0)) len = measured;
            const scale = len / measured;
            rulers.push({ id: uid(), ax: rulerStart.x, ay: RULER_Y, az: rulerStart.z, bx: rulerStart.x + dx * scale, by: RULER_Y, bz: rulerStart.z + dz * scale, length: Math.round(len) });
            rebuildRulers();
          }
        }
      }
    }
    rulerStart = null;
    return;
  }
  rulerStart = null;

  if (wasDrag) return; // 회전이었으면 무시
  setPointer(e);

  // 0) 배치(템플릿) 중: 고스트 자리에 배치 (연속)
  if (placing) {
    updatePlacementGhost();
    commitPlacing();
    updatePlacementGhost();   // 다음 배치용 고스트 갱신
    return;
  }

  // 0) 메모 모드: 부품 클릭 지점에 메모 추가
  if (mode === 'memo') {
    const hits = intersect(partsGroup.children);
    if (hits.length) { const p = hits[0].point; addNote(p.x, p.y, p.z); }
    else toast('조인트나 부품 위를 클릭하세요');
    return;
  }
  // 0) 장착 모드: 3단계 조인트 선택
  if (mode === 'mount') { mountClick(); return; }
  // 0) 흔들다리 모드: 매달 조인트 4개를 클릭하면 그 사각형에 매달림
  if (mode === 'bridge') {
    const hj = intersect(partsGroup.children.filter(m => m.userData.type === 'joint'));
    if (hj.length) {
      const id = hj[0].object.userData.id;
      if (!bridgePicks.includes(id)) bridgePicks.push(id);
      selection = bridgePicks.map(i => ({ type: 'joint', id: i }));   // 선택 강조
      if (bridgePicks.length >= 4) {
        const c = orderQuad(bridgePicks.slice(0, 4));
        if (!bridgeExists(c)) bridges.push({ id: uid(), c });
        bridgePicks = []; clearSel(); rebuild(); updateBOM(); toast('흔들다리를 연결했습니다');
      } else { rebuild(); toast(`매달 조인트 ${bridgePicks.length}/4 선택`); }
    } else {   // 흔들다리 클릭 → 선택 / 빈 곳 → 취소
      const hb = intersect(partsGroup.children.filter(m => m.userData.type === 'bridge'));
      if (hb.length) { bridgePicks = []; selectOne('bridge', hb[0].object.userData.id); rebuild(); updateBOM(); }
      else { bridgePicks = []; clearSel(); rebuild(); }
    }
    return;
  }
  // 0-1) 면 모드: 미리보기 클릭 → 판넬 채우기 / 판넬 클릭 → 선택
  if (mode === 'face') {
    const targets = facePreviewGroup.children.concat(partsGroup.children.filter(m => m.userData.type === 'panel'));
    const hits = intersect(targets);
    if (hits.length) {
      const o = hits[0].object;
      if (o.userData.type === 'facepreview') { const face = o.userData.face; if (!panelExists(face.c)) panels.push({ id: uid(), c: face.c.slice() }); clearSel(); }
      else selectOne('panel', o.userData.id);
      rebuild(); updateBOM();
      return;
    }
  }
  // 1) 파이프 모드: 화살표 클릭 → 파이프 추가
  if (mode === 'pipe' && handleGroup.visible && activeJoint) {
    const h = intersect(visibleHandleMeshes());
    if (h.length) {
      const j = getJoint(activeJoint.id);
      if (j) addInDirection(j, h[0].object.userData.dir);
      return;
    }
  }
  // 1b) 육면체 모드: 화살표 클릭 → 큐브 찍기
  if (mode === 'cube' && cubeGroup.visible && activeJoint) {
    const h = intersect(visibleCubeMeshes());
    if (h.length) {
      const j = getJoint(activeJoint.id);
      if (j) stampCube(j, h[0].object.userData.oct, activeLength, cubeHeight);
      return;
    }
  }
  // 2) 조인트 클릭 → 선택 + 방향 화살표 표시(유지)
  const hitJoint = intersect(partsGroup.children.filter(m => m.userData.type === 'joint'));
  if (hitJoint.length) {
    const id = hitJoint[0].object.userData.id;
    if (e.shiftKey) { toggleSel('joint', id); activeJoint = null; handleGroup.visible = false; cubeGroup.visible = false; rebuild(); updateBOM(); return; }
    selectOne('joint', id);
    activeJoint = { id };
    rebuild(); refreshHandles(); updateBOM();
    return;
  }
  // 2b) 판넬/흔들다리 클릭 → 선택 (조인트가 없을 때만 → 조인트 우선)
  const hitPB = intersect(partsGroup.children.filter(m => ['panel', 'bridge'].includes(m.userData.type)));
  if (hitPB.length) {
    const o = hitPB[0].object;
    if (e.shiftKey) toggleSel(o.userData.type, o.userData.id); else selectOne(o.userData.type, o.userData.id);
    activeJoint = null; handleGroup.visible = false; cubeGroup.visible = false;
    rebuild(); updateBOM();
    return;
  }
  // 3) 파이프 클릭 → 선택 (화살표 숨김)
  const hitPipe = intersect(partsGroup.children.filter(m => m.userData.type === 'pipe'));
  if (hitPipe.length) {
    const id = hitPipe[0].object.userData.id;
    if (e.shiftKey) toggleSel('pipe', id); else selectOne('pipe', id);
    activeJoint = null; handleGroup.visible = false; cubeGroup.visible = false;
    rebuild(); updateBOM();
    return;
  }
  // 3b) 육면체 모드: 빈 칸이면 바닥에, 기존 큐브가 있는 칸이면 그 위에 쌓기
  if (mode === 'cube') {
    const t = cubeGhostTarget();
    if (t) {
      stampCubeAt(t.ax, t.ay, t.az, activeLength, cubeHeight);
      clearSel(); activeJoint = null; handleGroup.visible = false; cubeGroup.visible = false;
      rebuild(); updateBOM();
      return;
    }
  }
  // 4) 빈 곳 클릭 → 선택 해제 + 화살표 숨김
  if (selection.length || handleGroup.visible || cubeGroup.visible) {
    clearSel(); activeJoint = null;
    handleGroup.visible = false; cubeGroup.visible = false;
    rebuild(); updateBOM();
  }
}

window.addEventListener('keydown', (e) => {
  const meta = e.ctrlKey || e.metaKey;
  if (meta && (e.key === 'z' || e.key === 'Z')) { e.preventDefault(); if (e.shiftKey) redo(); else undo(); return; }
  if (meta && (e.key === 'y' || e.key === 'Y')) { e.preventDefault(); redo(); return; }
  if (e.key === 'Delete' || e.key === 'Backspace') withUndo(deleteSelected);
});

// 방향 목록에 정반대(일직선) 쌍이 있으면 true
function hasColinearPair(dirs) {
  for (let a = 0; a < dirs.length; a++) for (let b = a + 1; b < dirs.length; b++)
    if (dirs[a].dot(dirs[b]) < -0.9) return true;
  return false;
}

// ===== 조인트 종류 판정 =====
function classifyJoint(jointId) {
  const dirs = jointDegreeDirs(jointId);
  const deg = dirs.length;
  if (deg === 0) return null;                     // 사용 안 됨
  if (deg === 1) return '마감캡';
  if (deg === 2) return dirs[0].dot(dirs[1]) < -0.9 ? '1자' : 'ㄱ자';   // 일직선 / 꺾임
  if (deg === 3) return hasColinearPair(dirs) ? 'T자' : '3구';           // 평면 T / 직각 코너
  return deg + '구';                              // 4구/5구/6구
}

// ===== BOM & 치수 계산 =====
function computeBOM() {
  const pipeCount = { 10: 0, 15: 0, 20: 0, 35: 0 };
  for (const p of pipes) pipeCount[p.length] = (pipeCount[p.length] || 0) + 1;

  const jointCount = {};
  for (const j of joints) {
    const type = classifyJoint(j.id);
    if (!type) continue;
    jointCount[type] = (jointCount[type] || 0) + 1;
  }

  const panelCount = {};   // 라벨 → 개수
  for (const panel of panels) {
    const [a, b] = panelSize(panel);
    const label = a === b ? `면 ${a}×${a}cm` : `면 ${a}×${b}cm`;
    panelCount[label] = (panelCount[label] || 0) + 1;
  }

  const bridgeCount = {};
  for (const bridge of bridges) {
    const [a, b] = bridgeSpan(bridge);
    const label = `흔들다리 ${a}×${b}cm`;
    bridgeCount[label] = (bridgeCount[label] || 0) + 1;
  }
  return { pipeCount, jointCount, panelCount, bridgeCount };
}

function computeDims() {
  if (joints.length === 0) return null;
  let min = { x: Infinity, y: Infinity, z: Infinity };
  let max = { x: -Infinity, y: -Infinity, z: -Infinity };
  for (const j of joints) {
    for (const ax of ['x', 'y', 'z']) {
      min[ax] = Math.min(min[ax], j[ax]);
      max[ax] = Math.max(max[ax], j[ax]);
    }
  }
  const span = { x: max.x - min.x, y: max.y - min.y, z: max.z - min.z };
  // 외곽 치수 = 중심 스팬 + 양끝 조인트 반경(2.5 x 2 = 5)
  const outer = { x: span.x + JOINT_SPAN, y: span.y + JOINT_SPAN, z: span.z + JOINT_SPAN };
  return { span, outer };
}

// 선택된 조인트 2개면 두 점 거리 표시
function updateDistInfo() {
  const el = document.getElementById('dist-info');
  if (!el) return;
  const js = selection.filter(s => s.type === 'joint').map(s => getJoint(s.id)).filter(Boolean);
  if (js.length === 2) {
    const [a, b] = js;
    const d = Math.round(Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
    const dx = Math.round(Math.abs(a.x - b.x)), dy = Math.round(Math.abs(a.y - b.y)), dz = Math.round(Math.abs(a.z - b.z));
    el.textContent = `📐 두 점 거리 ${d}cm  (가로 ${dx} · 높이 ${dy} · 세로 ${dz})`;
    el.style.display = 'block';
  } else el.style.display = 'none';
}

// ===== UI 렌더 =====
function updateBOM() {
  updateEditPanel();
  updateDistInfo();
  const { pipeCount, jointCount, panelCount, bridgeCount } = computeBOM();

  // 파이프
  const pipeEl = document.getElementById('bom-pipes');
  const JOINT_ORDER = ['마감캡', '1자', 'ㄱ자', 'T자', '3구', '4구', '5구', '6구'];
  let totalPipes = 0, totalJoints = 0, totalPanels = 0, totalBridges = 0, pipeTypes = 0, jointTypes = 0, panelTypes = 0, bridgeTypes = 0;
  let html = '<div class="bom-sub">파이프</div>';
  let anyPipe = false;
  for (const L of PIPE_LENGTHS) {
    const n = pipeCount[L] || 0;
    if (n === 0) continue;
    anyPipe = true; totalPipes += n; pipeTypes++;
    html += `<div class="bom-item"><span>파이프 ${L}cm</span><span class="qty">${n}</span></div>`;
  }
  if (!anyPipe) html += '<div class="empty">아직 없음</div>';
  pipeEl.innerHTML = html;

  // 조인트
  const jointEl = document.getElementById('bom-joints');
  let jhtml = '<div class="bom-sub">조인트</div>';
  let anyJoint = false;
  for (const t of JOINT_ORDER) {
    const n = jointCount[t] || 0;
    if (n === 0) continue;
    anyJoint = true; totalJoints += n; jointTypes++;
    jhtml += `<div class="bom-item"><span>${t}</span><span class="qty">${n}</span></div>`;
  }
  if (!anyJoint) jhtml += '<div class="empty">아직 없음</div>';
  jointEl.innerHTML = jhtml;

  // 판넬(면)
  const panelEl = document.getElementById('bom-panels');
  let phtml = '<div class="bom-sub">면(판넬)</div>';
  const panelLabels = Object.keys(panelCount).sort();
  for (const label of panelLabels) {
    const n = panelCount[label];
    totalPanels += n; panelTypes++;
    phtml += `<div class="bom-item"><span>${label}</span><span class="qty">${n}</span></div>`;
  }
  if (panelLabels.length === 0) phtml += '<div class="empty">아직 없음</div>';
  panelEl.innerHTML = phtml;

  // 흔들다리
  const bridgeEl = document.getElementById('bom-bridges');
  const bridgeLabels = Object.keys(bridgeCount).sort();
  if (bridgeLabels.length) {
    let bhtml = '<div class="bom-sub">흔들다리</div>';
    for (const label of bridgeLabels) {
      const n = bridgeCount[label];
      totalBridges += n; bridgeTypes++;
      bhtml += `<div class="bom-item"><span>${label}</span><span class="qty">${n}</span></div>`;
    }
    bridgeEl.innerHTML = bhtml;
  } else bridgeEl.innerHTML = '';

  // 합계 (총 수 + 종류 수)
  document.getElementById('bom-total').innerHTML =
    `<div class="total-line"><span>총 부품 수</span><span>${totalPipes + totalJoints + totalPanels + totalBridges}개</span></div>` +
    `<div class="total-line sub"><span>부품 종류</span><span>${pipeTypes + jointTypes + panelTypes + bridgeTypes}종류</span></div>`;

  // 치수
  const dimsEl = document.getElementById('dims');
  const d = computeDims();
  if (!d) {
    dimsEl.innerHTML = '<div class="empty">부품을 추가하세요</div>';
    return;
  }
  const f = (v) => Math.round(v * 10) / 10;
  dimsEl.innerHTML = `
    <div class="headline">${f(d.outer.x)} × ${f(d.outer.z)} × ${f(d.outer.y)} cm</div>
    <div class="axis">가로(X) 중심길이 <b>${f(d.span.x)}</b> cm</div>
    <div class="axis">세로(Z) 중심길이 <b>${f(d.span.z)}</b> cm</div>
    <div class="axis">높이(Y) 중심길이 <b>${f(d.span.y)}</b> cm</div>
  `;
}

// ===== 파이프 길이 버튼 =====
function buildPipeButtons() {
  const wrap = document.getElementById('pipe-buttons');
  wrap.innerHTML = '';
  for (const L of PIPE_LENGTHS) {
    const b = document.createElement('button');
    b.className = 'pipe-btn' + (L === activeLength ? ' active' : '');
    b.innerHTML = `${L}cm<small>중심간 ${L + JOINT_SPAN}cm</small>`;
    b.addEventListener('click', () => {
      activeLength = L;
      buildPipeButtons();
      refreshHandles();   // 육면체 고스트 크기 갱신
    });
    wrap.appendChild(b);
  }
}

// ===== 육면체 높이 버튼 =====
function buildHeightButtons() {
  const wrap = document.getElementById('height-buttons');
  wrap.innerHTML = '';
  for (const H of PIPE_LENGTHS) {
    const b = document.createElement('button');
    b.className = 'height-btn' + (H === cubeHeight ? ' active' : '');
    b.textContent = `${H}cm`;
    b.addEventListener('click', () => {
      cubeHeight = H;
      buildHeightButtons();
      refreshHandles();   // 육면체 고스트 높이 갱신
    });
    wrap.appendChild(b);
  }
}

// ===== 선택 파이프 길이 변경 버튼 =====
function buildEditButtons() {
  const wrap = document.getElementById('edit-len-buttons');
  wrap.innerHTML = '';
  for (const L of PIPE_LENGTHS) {
    const b = document.createElement('button');
    b.className = 'height-btn';
    b.textContent = `${L}cm`;
    b.addEventListener('click', () => withUndo(() => setSelectedPipesLength(L)));
    wrap.appendChild(b);
  }
}
function updateEditPanel() {
  const selPipes = selection.filter(s => s.type === 'pipe');
  const sec = document.getElementById('edit-section');
  if (!sec) return;
  sec.style.display = selPipes.length ? 'block' : 'none';
  if (!selPipes.length) return;
  // 선택된 파이프들의 현재 길이 표기 (길이별 개수)
  const byLen = {};
  for (const s of selPipes) { const p = pipes.find(x => x.id === s.id); if (p) byLen[p.length] = (byLen[p.length] || 0) + 1; }
  const parts = Object.keys(byLen).map(Number).sort((a, b) => a - b).map(L => byLen[L] > 1 ? `${L}cm×${byLen[L]}` : `${L}cm`);
  const lenText = parts.join(', ');
  document.getElementById('edit-count').textContent = selPipes.length === 1
    ? `선택한 파이프: ${lenText}`
    : `선택 파이프 ${selPipes.length}개 (${lenText})`;
}

// ===== 저장 / 불러오기 / 복사 / 초기화 =====
const LAST_KEY = 'junglegym:last';   // 마지막 불러온/저장한 설계 (localStorage)

function currentDesign() {
  return { joints, pipes, panels, bridges, notes, rulers, groups, activeLength, nextId };
}
// localStorage에 최근 설계 기억
function rememberLast(name, d) {
  try { localStorage.setItem(LAST_KEY, JSON.stringify({ name, d })); } catch (e) {}
  updateLastButton();
}
function updateLastButton() {
  const b = document.getElementById('btn-load-last');
  if (!b) return;
  let entry = null;
  try { entry = JSON.parse(localStorage.getItem(LAST_KEY)); } catch (e) {}
  b.disabled = !entry;
  b.textContent = entry ? `🕘 ${entry.name || '최근 설계'} 다시` : '🕘 최근 없음';
}
// 설계 객체를 화면에 적용
function applyLoaded(d) {
  joints = d.joints || [];
  pipes = d.pipes || [];
  panels = d.panels || [];
  bridges = d.bridges || [];
  notes = d.notes || [];
  rulers = d.rulers || [];
  groups = d.groups || [];
  activeLength = d.activeLength || 20;
  nextId = d.nextId || (Math.max(0, ...joints.map(j => j.id), ...pipes.map(p => p.id), ...panels.map(p => p.id), ...bridges.map(b => b.id), ...notes.map(n => n.id), ...rulers.map(r => r.id)) + 1);
  clearSel(); activeJoint = null; handleGroup.visible = false; cubeGroup.visible = false;
  buildPipeButtons(); rebuild(); updateBOM(); renderNotes(); rebuildRulers();
}
// 최근 설계 바로 불러오기 (파일 선택 없이)
function loadLast() {
  let entry = null;
  try { entry = JSON.parse(localStorage.getItem(LAST_KEY)); } catch (e) {}
  if (!entry || !entry.d) { toast('최근 불러온 파일이 없습니다'); return; }
  const b = snapState(); applyLoaded(entry.d); commitUndo(b);
  toast(`${entry.name || '최근 설계'} 불러옴`);
}
const FS_SUPPORTED = 'showSaveFilePicker' in window;
let fileHandle = null;   // 현재 열려있는 JSON 파일 핸들 (덮어쓰기 대상)

function fileLabel() {
  const el = document.getElementById('file-label');
  if (el) el.textContent = fileHandle ? `현재 파일: ${fileHandle.name} (저장 시 덮어씀)` : (FS_SUPPORTED ? '열린 파일 없음 (저장 시 위치 선택)' : '');
}
function downloadJSON(data) {
  const blob = new Blob([data], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'junglegym-design.json';
  a.click();
  URL.revokeObjectURL(a.href);
}
// 저장: 열린 파일이 있으면 덮어쓰기, 없으면 위치 선택(이후 그 파일에 덮어씀). 미지원 시 다운로드.
async function saveJSON() {
  const d = currentDesign();
  const data = JSON.stringify(d, null, 2);
  if (FS_SUPPORTED) {
    try {
      if (!fileHandle) {
        fileHandle = await window.showSaveFilePicker({ suggestedName: 'junglegym-design.json', types: [{ description: 'JSON', accept: { 'application/json': ['.json'] } }] });
      }
      const w = await fileHandle.createWritable();
      await w.write(data); await w.close();
      rememberLast(fileHandle.name, d); fileLabel();
      toast(`덮어쓰기 저장: ${fileHandle.name}`);
      return;
    } catch (e) { if (e.name === 'AbortError') return; }   // 취소면 중단
  }
  downloadJSON(data); rememberLast('junglegym-design.json', d);
  toast('JSON 저장 완료');
}
// 불러오기: 파일 핸들 확보 → 이후 저장이 그 파일을 덮어씀. 미지원 시 파일 입력.
async function openJSON() {
  if (FS_SUPPORTED) {
    try {
      const [h] = await window.showOpenFilePicker({ types: [{ description: 'JSON', accept: { 'application/json': ['.json'] } }] });
      fileHandle = h;
      const file = await h.getFile();
      const d = JSON.parse(await file.text());
      const b = snapState(); applyLoaded(d); commitUndo(b); rememberLast(h.name, d); fileLabel();
      toast(`불러오기: ${h.name}`);
      return;
    } catch (e) { if (e.name === 'AbortError') return; toast('파일을 읽을 수 없습니다'); return; }
  }
  document.getElementById('file-input').click();
}
function importJSON(file) {   // 폴백(파일 입력) 경로 — 핸들 없음
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const d = JSON.parse(reader.result);
      const b = snapState(); applyLoaded(d); commitUndo(b);
      rememberLast(file.name, d);
      fileHandle = null; fileLabel();
      toast('불러오기 완료');
    } catch (err) {
      toast('파일을 읽을 수 없습니다');
    }
  };
  reader.readAsText(file);
}
function copyBOM() {
  const { pipeCount, jointCount, panelCount, bridgeCount } = computeBOM();
  const d = computeDims();
  let total = 0, types = 0;
  let txt = '[정글짐 부품표]\n\n파이프\n';
  for (const L of PIPE_LENGTHS) if (pipeCount[L]) { txt += `  ${L}cm x ${pipeCount[L]}\n`; total += pipeCount[L]; types++; }
  txt += '\n조인트\n';
  for (const t of ['마감캡', '1자', 'ㄱ자', 'T자', '3구', '4구', '5구', '6구'])
    if (jointCount[t]) { txt += `  ${t} x ${jointCount[t]}\n`; total += jointCount[t]; types++; }
  const pLabels = Object.keys(panelCount).sort();
  if (pLabels.length) {
    txt += '\n면(판넬)\n';
    for (const label of pLabels) { txt += `  ${label} x ${panelCount[label]}\n`; total += panelCount[label]; types++; }
  }
  const bLabels = Object.keys(bridgeCount).sort();
  if (bLabels.length) {
    txt += '\n흔들다리\n';
    for (const label of bLabels) { txt += `  ${label} x ${bridgeCount[label]}\n`; total += bridgeCount[label]; types++; }
  }
  txt += `\n총 부품 수: ${total}개 (${types}종류)\n`;
  if (d) {
    const f = (v) => Math.round(v * 10) / 10;
    txt += `\n최종 치수: ${f(d.outer.x)} x ${f(d.outer.z)} x ${f(d.outer.y)} cm (가로x세로x높이)\n`;
  }
  if (notes.length) { txt += '\n메모\n'; for (const n of notes) txt += `  - ${n.text}\n`; }
  const done = () => toast('BOM 복사 완료');
  if (navigator.clipboard) {
    navigator.clipboard.writeText(txt).then(done).catch(() => fallbackCopy(txt, done));
  } else fallbackCopy(txt, done);
}
function fallbackCopy(txt, cb) {
  const ta = document.createElement('textarea');
  ta.value = txt; document.body.appendChild(ta); ta.select();
  try { document.execCommand('copy'); cb(); } catch (e) { toast('복사 실패'); }
  document.body.removeChild(ta);
}
// ===== 메모 =====
function addNote(x, y, z) {
  const text = prompt('메모 내용을 입력하세요');
  if (text == null || text.trim() === '') return;
  notes.push({ id: uid(), x, y, z, text: text.trim() });
  renderNotes();
}
function renderNotes() {
  noteLayer.innerHTML = '';
  for (const note of notes) {
    const el = document.createElement('div');
    el.className = 'memo-label';
    el.dataset.id = note.id;
    const span = document.createElement('span');
    span.className = 'memo-text';
    span.textContent = note.text;
    span.title = '클릭하면 수정';
    span.addEventListener('click', () => {
      const t = prompt('메모 수정', note.text);
      if (t != null) withUndo(() => { note.text = t.trim(); renderNotes(); });
    });
    const del = document.createElement('button');
    del.className = 'memo-del';
    del.textContent = '×';
    del.title = '삭제';
    del.addEventListener('click', () => withUndo(() => { notes = notes.filter(n => n.id !== note.id); renderNotes(); }));
    el.appendChild(span); el.appendChild(del);
    noteLayer.appendChild(el);
  }
  positionNotes();
}
const _npos = new THREE.Vector3();
function positionNotes() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  for (const el of noteLayer.children) {
    const note = notes.find(n => n.id === Number(el.dataset.id));
    if (!note) continue;
    _npos.set(note.x, note.y, note.z).project(activeCam);
    if (_npos.z > 1) { el.style.display = 'none'; continue; }  // 카메라 뒤
    el.style.display = 'flex';
    el.style.left = ((_npos.x * 0.5 + 0.5) * w) + 'px';
    el.style.top = ((-_npos.y * 0.5 + 0.5) * h) + 'px';
  }
}

// ===== 템플릿 / 크기 맞춤 생성 =====
// 셀 좌표계: 한 칸 = (파이프 L + 조인트 5cm) 간격. cell [i,j,k] = x/높이(y)/z 번째 칸.
function clearDesign() {
  joints = []; pipes = []; panels = []; bridges = []; notes = []; groups = []; nextId = 1; selection = []; activeJoint = null;
  handleGroup.visible = false; cubeGroup.visible = false;
  renderNotes();
}
function confirmReplace() {
  return pipes.length === 0 || confirm('현재 설계를 지우고 새로 만들까요?');
}
// 셀의 한 면(수평 top/bottom + 수직 x±/z±)을 이루는 4조인트 id (원점 오프셋 지원)
function faceJointIds(cell, face, s, ox = 0, oy = 0, oz = 0) {
  const [i, j, k] = cell;
  let coords;
  if (face === 'top' || face === 'bottom') {
    const y = oy + (face === 'top' ? j + 1 : j) * s;
    coords = [[ox + i * s, y, oz + k * s], [ox + (i + 1) * s, y, oz + k * s], [ox + (i + 1) * s, y, oz + (k + 1) * s], [ox + i * s, y, oz + (k + 1) * s]];
  } else if (face === 'x+' || face === 'x-') {
    const x = ox + (face === 'x+' ? i + 1 : i) * s;
    coords = [[x, oy + j * s, oz + k * s], [x, oy + (j + 1) * s, oz + k * s], [x, oy + (j + 1) * s, oz + (k + 1) * s], [x, oy + j * s, oz + (k + 1) * s]];
  } else {  // 'z+' | 'z-'
    const z = oz + (face === 'z+' ? k + 1 : k) * s;
    coords = [[ox + i * s, oy + j * s, z], [ox + (i + 1) * s, oy + j * s, z], [ox + (i + 1) * s, oy + (j + 1) * s, z], [ox + i * s, oy + (j + 1) * s, z]];
  }
  const cs = coords.map(c => findJointNear(c[0], c[1], c[2]));
  if (cs.some(c => !c)) return null;
  const ids = cs.map(c => c.id);
  for (let n = 0; n < 4; n++) if (!pipeBetween(ids[n], ids[(n + 1) % 4])) return null;
  return ids;
}
function addFacePanel(cell, face, s, ox, oy, oz) {
  const ids = faceJointIds(cell, face, s, ox, oy, oz);
  if (ids && !panelExists(ids)) panels.push({ id: uid(), c: ids });
}
function addFaceBridge(cell, face, s, ox, oy, oz) {
  const ids = faceJointIds(cell, face, s, ox, oy, oz);
  if (ids && !bridgeExists(ids)) bridges.push({ id: uid(), c: ids });
}

// cell = [i(가로x), j(높이y), k(세로z)]. panels/bridges 는 {cell, face}
const TEMPLATES = [
  { key: 'cube',   name: '큐브 1칸',        cells: [[0,0,0]] },
  { key: 'grid',   name: '정글짐 2×2×2',    cells: (() => { const c = []; for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) for (let d = 0; d < 2; d++) c.push([a, b, d]); return c; })() },
  { key: 'big',    name: '큰 정글짐 3×2×2', cells: (() => { const c = []; for (let a = 0; a < 3; a++) for (let b = 0; b < 2; b++) for (let d = 0; d < 2; d++) c.push([a, b, d]); return c; })(),
    panels: [{ cell: [0,1,0], face: 'top' }, { cell: [1,1,0], face: 'top' }, { cell: [2,1,0], face: 'top' }, { cell: [0,1,1], face: 'top' }, { cell: [1,1,1], face: 'top' }, { cell: [2,1,1], face: 'top' }] },
  { key: 'plate',  name: '놀이판 2×2',      cells: [[0,0,0],[1,0,0],[0,0,1],[1,0,1]],
    panels: [{ cell: [0,0,0], face: 'top' }, { cell: [1,0,0], face: 'top' }, { cell: [0,0,1], face: 'top' }, { cell: [1,0,1], face: 'top' }] },
  { key: 'tower',  name: '타워 3층',        cells: [[0,0,0],[0,1,0],[0,2,0]],
    panels: [{ cell: [0,1,0], face: 'top' }, { cell: [0,2,0], face: 'top' }] },
  { key: 'Lshape', name: 'ㄱ자 2층',        cells: [[0,0,0],[0,1,0],[1,0,0],[1,1,0],[0,0,1],[0,1,1]],
    panels: [{ cell: [0,1,0], face: 'top' }, { cell: [1,1,0], face: 'top' }, { cell: [0,1,1], face: 'top' }] },
  { key: 'house',  name: '놀이집(벽+지붕)', cells: [[0,0,0],[0,1,0]],
    panels: [{ cell: [0,1,0], face: 'top' }, { cell: [0,0,0], face: 'z-' }, { cell: [0,1,0], face: 'z-' }, { cell: [0,0,0], face: 'x-' }, { cell: [0,1,0], face: 'x-' }] },
  { key: 'twin',   name: '트윈타워 + 다리', cells: [[0,0,0],[0,1,0],[2,0,0],[2,1,0],[1,1,0]],
    panels: [{ cell: [1,1,0], face: 'bottom' }, { cell: [0,1,0], face: 'top' }, { cell: [2,1,0], face: 'top' }] },
  { key: 'ropebridge', name: '출렁다리 세트', cells: [[0,0,0],[0,1,0],[2,0,0],[2,1,0],[1,1,0]],
    panels: [{ cell: [0,1,0], face: 'top' }, { cell: [2,1,0], face: 'top' }],
    bridges: [{ cell: [1,1,0], face: 'bottom' }] },
  { key: 'stairs', name: '계단',            cells: [[0,0,0],[1,0,0],[1,1,0],[2,0,0],[2,1,0],[2,2,0]],
    panels: [{ cell: [0,0,0], face: 'top' }, { cell: [1,1,0], face: 'top' }, { cell: [2,2,0], face: 'top' }] },
];

// ===== 고스트 배치 (템플릿/육면체를 커서로 옮겨 스냅 후 클릭 배치) =====
let placing = null;   // { cells, panels, label, _anchor }

function startPlacing(item) {
  if (placing && placing.label === item.label) { stopPlacing(); return; }  // 같은 버튼 다시 → 취소
  placing = { ...item, _anchor: null };
  document.getElementById('place-banner').style.display = 'flex';
  document.getElementById('place-banner-text').textContent =
    `${item.label} 배치 — 조인트에 대면 그 지점에, 바닥은 격자에 붙습니다. 클릭 배치 · Esc/우클릭 종료`;
  updateToolButtons();
}
function stopPlacing() {
  placing = null;
  clearGhost(); ghostGroup.visible = false;
  document.getElementById('place-banner').style.display = 'none';
  updateToolButtons();
}
function updateToolButtons() {
  document.querySelectorAll('.tpl-btn').forEach(b => b.classList.toggle('active', !!placing && placing.label === b.dataset.label));
}
// 현재 커서 기준 스냅 앵커 계산 + 고스트 갱신
function updatePlacementGhost() {
  if (!placing) return;
  const s = activeLength + JOINT_SPAN;
  const hitJoint = intersect(partsGroup.children.filter(m => m.userData.type === 'joint'));
  let ax, ay, az;
  if (hitJoint.length) {   // 조인트에 스냅 (높이 포함)
    const j = getJoint(hitJoint[0].object.userData.id);
    ax = j.x; ay = j.y; az = j.z;
  } else {                 // 바닥 격자에 스냅
    const gp = groundPoint();
    if (!gp) { placing._anchor = null; clearGhost(); ghostGroup.visible = false; return; }
    ax = snapGrid(gp.x, s); ay = 0; az = snapGrid(gp.z, s);
  }
  placing._anchor = { x: ax, y: ay, z: az };
  showGhost(placing.cells, ax, ay, az, s);
}
function commitPlacing() {
  if (!placing || !placing._anchor) return;
  const s = activeLength + JOINT_SPAN;
  const { x: ax, y: ay, z: az } = placing._anchor;
  for (const [i, j, k] of placing.cells) stampCubeAt(ax + i * s, ay + j * s, az + k * s, activeLength);
  for (const f of (placing.panels || [])) addFacePanel(f.cell, f.face, s, ax, ay, az);
  for (const f of (placing.bridges || [])) addFaceBridge(f.cell, f.face, s, ax, ay, az);
  rebuild(); updateBOM();
}

// 목표 길이(cm)에 가장 가까운 (칸 수 n, 파이프 길이 L) 조합. 외곽 = n*(L+5)+5
function bestFit(target) {
  let best = null;
  for (const L of PIPE_LENGTHS) {
    const spacing = L + JOINT_SPAN;
    const guess = Math.round((target - JOINT_SPAN) / spacing);
    for (const n of [guess - 1, guess, guess + 1]) {
      if (n < 1 || n > 6) continue;
      const size = n * spacing + JOINT_SPAN;
      const err = Math.abs(size - target);
      if (!best || err < best.err) best = { n, L, size, err };
    }
  }
  return best;
}

function generateBySize() {
  const W = parseFloat(document.getElementById('size-w').value);
  const D = parseFloat(document.getElementById('size-d').value);
  const H = parseFloat(document.getElementById('size-h').value);
  if (!(W > 0 && D > 0 && H > 0)) { toast('가로·세로·높이를 모두 입력하세요'); return; }
  const fx = bestFit(W), fz = bestFit(D), fy = bestFit(H);
  if (!fx || !fz || !fy) { toast('만들 수 있는 크기가 아닙니다 (최소 15cm)'); return; }
  if (!confirmReplace()) return;
  clearDesign();
  const sx = fx.L + JOINT_SPAN, sy = fy.L + JOINT_SPAN, sz = fz.L + JOINT_SPAN;
  // 축별 파이프 길이가 달라도 되도록 격자를 직접 구성
  const jid = {};
  for (let i = 0; i <= fx.n; i++) for (let j = 0; j <= fy.n; j++) for (let k = 0; k <= fz.n; k++)
    jid[`${i},${j},${k}`] = addJoint(i * sx, j * sy, k * sz).id;
  const link = (a, b, length, axis) => pipes.push({ id: uid(), a, b, length, axis });
  for (let i = 0; i <= fx.n; i++) for (let j = 0; j <= fy.n; j++) for (let k = 0; k <= fz.n; k++) {
    if (i < fx.n) link(jid[`${i},${j},${k}`], jid[`${i + 1},${j},${k}`], fx.L, 'x');
    if (j < fy.n) link(jid[`${i},${j},${k}`], jid[`${i},${j + 1},${k}`], fy.L, 'y');
    if (k < fz.n) link(jid[`${i},${j},${k}`], jid[`${i},${j},${k + 1}`], fz.L, 'z');
  }
  rebuild(); updateBOM(); applyView();
  toast(`생성 완료: ${fx.size} × ${fz.size} × ${fy.size} cm (${fx.n}×${fz.n}×${fy.n}칸)`);
}

function resetAll() {
  if (!confirm('모든 부품을 삭제하고 처음부터 시작할까요? (줄자 포함)')) return;
  clearDesign();
  rulers = []; rebuildRulers();
  addJoint(0, 0, 0);   // 시드
  rebuild(); updateBOM();
}

let toastTimer = null;
function toast(msg) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 1600);
}

// 보기(2D/3D) 전환
document.querySelectorAll('.view-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    view = btn.dataset.view;
    document.querySelectorAll('.view-btn').forEach(b => b.classList.toggle('active', b === btn));
    document.getElementById('plane-row').style.display = view === '2d' ? 'flex' : 'none';
    if (view === '2d') {   // 2D 진입 시 항상 탑뷰(평면도)로 시작
      plane = 'top';
      document.querySelectorAll('.plane-btn').forEach(b => b.classList.toggle('active', b.dataset.plane === 'top'));
    }
    applyView();
  });
});
// 2D 평면 전환
document.querySelectorAll('.plane-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    plane = btn.dataset.plane;
    document.querySelectorAll('.plane-btn').forEach(b => b.classList.toggle('active', b === btn));
    if (view === '2d') applyView();
  });
});

// 모드 전환
document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    mode = btn.dataset.mode;
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.toggle('active', b === btn));
    document.getElementById('len-hint').textContent =
      mode === 'pipe' ? '길이를 고르고, 조인트를 클릭하면 방향 화살표가 나옵니다. 화살표를 클릭해 연결하세요.'
      : mode === 'cube' ? '빈 바닥을 클릭해 육면체를 놓고, 기존 육면체 위를 가리키면 그 위에 쌓입니다. 조인트 클릭 시 방향 화살표로도 붙일 수 있어요.'
      : mode === 'face' ? '파이프 4개가 사각형을 이룬 면이 반투명으로 표시됩니다. 클릭하면 판넬이 채워집니다.'
      : mode === 'bridge' ? '흔들다리를 매달 조인트 4개를 클릭하면 그 사각형에 밧줄 다리가 매달립니다.'
      : mode === 'memo' ? '부품(조인트·파이프)을 클릭하면 그 위치에 메모를 남길 수 있습니다.'
      : mode === 'ruler' ? '바닥을 드래그해 선을 긋고 길이(cm)를 입력하면 초록 줄자가 표시됩니다. (전체 크기 가늠용)'
      : mode === 'mount' ? '그룹을 구조에 기울여 붙입니다. 아래 단계 안내를 따라 조인트를 클릭하세요.'
      : '드래그로 여러 개 선택 · 클릭/Shift+클릭으로 선택 · 선택된 것을 드래그하면 이동 · Delete로 삭제 · 여러 개 골라 "그룹 묶기". (3D는 우클릭 드래그로 회전)';
    handleGroup.visible = false;
    cubeGroup.visible = false;
    activeJoint = null;
    stopPlacing();          // 모드 전환 시 배치 취소
    clearGhost(); ghostGroup.visible = false;
    hideSelectBox(); hideRulerPreview(); rulerStart = null; moving = null; bridgePicks = [];
    mountReset();
    document.getElementById('ruler-section').style.display = mode === 'ruler' ? 'block' : 'none';
    document.getElementById('mount-section').style.display = mode === 'mount' ? 'block' : 'none';
    document.getElementById('group-row').style.display = mode === 'select' ? 'flex' : 'none';
    applyControlButtons();  // 선택 모드 진입/이탈 시 마우스 버튼 매핑 갱신
    refreshHandles();
    rebuildFacePreviews();   // 면 모드 진입 시 미리보기 갱신
    if (mode === 'face') {
      const n = facePreviewGroup.children.length;
      if (n) toast(`채울 수 있는 면 ${n}곳`);
    }
  });
});

// 버튼 바인딩
// 템플릿 버튼 → 고스트 배치 시작
{
  const wrap = document.getElementById('template-buttons');
  for (const t of TEMPLATES) {
    const b = document.createElement('button');
    b.className = 'tpl-btn';
    b.dataset.label = t.name;
    b.textContent = t.name;
    b.addEventListener('click', () => startPlacing({ cells: t.cells, panels: t.panels, bridges: t.bridges, label: t.name }));
    wrap.appendChild(b);
  }
}
// Esc → 배치 취소
window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && placing) stopPlacing(); });
// 우클릭 → 배치 취소 (컨텍스트 메뉴 대신)
renderer.domElement.addEventListener('contextmenu', (e) => { if (placing) { e.preventDefault(); stopPlacing(); } });
document.getElementById('btn-generate').addEventListener('click', () => withUndo(generateBySize));

document.getElementById('btn-delete').addEventListener('click', () => withUndo(deleteSelected));
document.getElementById('btn-reset').addEventListener('click', () => withUndo(resetAll));
document.getElementById('btn-export').addEventListener('click', saveJSON);
document.getElementById('btn-copy').addEventListener('click', copyBOM);
document.getElementById('btn-import').addEventListener('click', openJSON);
document.getElementById('btn-load-last').addEventListener('click', loadLast);
document.getElementById('btn-group').addEventListener('click', () => withUndo(groupSelection));
document.getElementById('btn-ungroup').addEventListener('click', () => withUndo(ungroupSelection));
document.getElementById('btn-mount-next').addEventListener('click', mountNext);
// 줄자 방향 토글
document.querySelectorAll('.ruler-dir-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    rulerDir = btn.dataset.dir;
    document.querySelectorAll('.ruler-dir-btn').forEach(b => b.classList.toggle('active', b === btn));
    document.getElementById('ruler-dir-hint').textContent =
      rulerDir === 'h' ? '바닥을 드래그해 선을 긋고 길이를 입력하세요.'
      : rulerDir === 'a' ? '바닥을 드래그하면 X/Z축에 맞춰 곧게(직각) 그어집니다.'
      : '바닥의 한 점을 클릭하고 높이(cm)를 입력하면 위로 섭니다.';
    hideRulerPreview(); rulerStart = null;
  });
});
document.getElementById('file-input').addEventListener('change', (e) => {
  if (e.target.files[0]) importJSON(e.target.files[0]);
  e.target.value = '';
});

// ===== 리사이즈 & 루프 =====
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  const aspect = w / h;
  orthoCam.left = -VIEW2D_SIZE * aspect; orthoCam.right = VIEW2D_SIZE * aspect;
  orthoCam.top = VIEW2D_SIZE; orthoCam.bottom = -VIEW2D_SIZE;
  orthoCam.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, activeCam);
  positionNotes();
  positionRulerLabels();
}

// ===== 시작 =====
addJoint(0, 0, 0);   // 원점 시드 조인트
buildPipeButtons();
buildHeightButtons();
buildEditButtons();
updateLastButton();
fileLabel();
rebuild();
updateBOM();
resize();
animate();

// PWA 서비스워커 등록 (오프라인 지원)
if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => navigator.serviceWorker.register('sw.js').catch(() => {}));
}
