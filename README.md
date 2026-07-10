# 정글짐 3D 설계 (taeri-jungle)

아이 정글짐(파이프 + 조인트 조립식)을 웹에서 3D로 설계하고, 최종 치수와
사용 부품 개수(BOM)를 확인하는 도구입니다. 빌드 없이 `index.html`만 열면 동작하며,
PWA로 아이패드 홈 화면에 설치할 수 있습니다.

## 실행
- 로컬: `index.html`을 브라우저로 열거나 정적 서버로 서빙
- 온라인: GitHub Pages 등 HTTPS 호스팅 → Safari에서 접속 → **공유 → 홈 화면에 추가**

## 주요 기능
- 파이프(10/15/25/35cm)·조인트 배치, 육면체 그리기·쌓기, 면/흔들다리
- 2D 그리기 ↔ 3D 뷰, 템플릿·크기 기반 자동 생성
- 다중 선택/삭제, 길이 변경, 그룹, 장착(연결), 줄자, 거리 측정, 메모
- 실시간 치수·BOM, JSON 저장/불러오기, 되돌리기(Ctrl+Z)
- PWA(오프라인 지원)

## 구성
- `index.html` · `style.css` · `app.js` — 앱 본체
- `manifest.webmanifest` · `sw.js` · `icon-*.png` — PWA
- `vendor/` — Three.js r128 + OrbitControls (로컬 번들)
