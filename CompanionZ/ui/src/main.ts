import './style.css';
import { h, s, clear } from './dom';
import { onStatus, startLink } from './api';
import { dashboard } from './pages/dashboard';
import { settings } from './pages/settings';
import { calibration } from './pages/calibration';
import { wifi } from './pages/wifi';

export type Page = (root: HTMLElement) => () => void;
const routes: [string, string, Page][] = [
  ['dashboard', 'Dashboard', dashboard],
  ['settings', 'Settings', settings],
  ['calibration', 'Calibration', calibration],
  ['wifi', 'WiFi', wifi],
];

const badge = h('span', { class: 'badge', 'data-state': 'OFFLINE' }, h('i'), h('b', null, 'OFFLINE'));
const meta = h('span', { class: 'meta mono' });
const nav = h('nav', { 'aria-label': 'Sections' }, routes.map(([id, label]) => h('a', { href: '#' + id, 'data-id': id }, label)));
const main = h('main');
document.getElementById('app')!.append(
  h('header', null,
    h('div', { class: 'bar' },
      h('div', { class: 'brand' }, s('svg', { viewBox: '0 0 16 16', width: 16, height: 16, 'aria-hidden': 'true' }, s('path', { d: 'M2 1h12v2H6l8 10v2H2v-2h8L2 3z' })), 'Z-BRO'),
      meta, badge),
    nav),
  main);
onStatus(s => {
  const st = s ? s.state : 'OFFLINE';
  badge.dataset.state = st;
  badge.lastElementChild!.textContent = st;
  meta.textContent = s ? `${s.fw || 'fw ?'}  ${s.rssi} dBm${s.s3 ? '' : '  S3 away'}` : 'no link';
});

let dispose: (() => void) | null = null;
function route() {
  const id = location.hash.slice(1) || 'dashboard';
  const r = routes.find(x => x[0] === id) ?? routes[0];
  nav.querySelectorAll('a').forEach(a => a.toggleAttribute('aria-current', a.dataset.id === r[0]));
  dispose?.();
  clear(main);
  const page = h('div', { class: 'page ' + r[0] });
  main.appendChild(page);
  dispose = r[2](page);
  window.scrollTo(0, 0);
}
window.addEventListener('hashchange', route);
route();
startLink();
