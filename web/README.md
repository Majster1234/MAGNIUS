# Panel robota

React, TypeScript i Vite. Wszystkie zasoby są lokalne, bez CDN i zewnętrznych
fontów. Backend udostępnia stronę i WebSocket pod tym samym originem.

```bash
npm ci
npm run build
npm test
```

Node.js 22.12+ (linia 22). Zależności są przypięte w `package-lock.json`.
`dist/` jest ignorowany przez Git i instalowany przez `robot_web_bridge`
podczas budowania workspace. `npm run dev` służy edycji wyglądu na localhost;
pełne połączenie z ROS sprawdzaj na zbudowanym panelu pod adresem LAN.

Nieaktualne dane są wykrywane według wieku zgłoszonego przez backend oraz
lokalnego czasu monotonicznego. Po zerwaniu WebSocket panel ponawia połączenie.
Nie ma pozycji zastępczych ani przycisków sterowania napędami.

Instrukcja całego systemu: [docs/web_status.md](../docs/web_status.md).
