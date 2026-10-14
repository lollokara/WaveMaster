import { h, s, toast } from '../dom';
import { api, ApiError, Cal, CalTest, Trial, onCal, onStatus, Status } from '../api';

const LABEL: Record<string, [string, string]> = {
  endstop: ['Endstop', 'Probe repeatability, sets the debounce time'],
  speed: ['Speed', 'Highest speed without lost steps'],
  accel: ['Acceleration', 'Steepest ramp up without lost steps'],
  decel: ['Deceleration', 'Steepest ramp down without lost steps'],
  homing: ['Homing', 'Fastest approach that still stops inside the back-off'],
};
const nm = (n: string) => LABEL[n]?.[0] ?? n;
const num = (v: number) => (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1));

function chart(t: CalTest, W: number): SVGElement {
  const Hh = 96, L = 34, R = 8, T = 8, B = 16;
  const svg = s('svg', { viewBox: `0 0 ${W} ${Hh}`, width: W, height: Hh, class: 'chart', role: 'img', 'aria-label': `${nm(t.name)} trials` });
  const tr: Trial[] = t.trials;
  if (!tr.length) { svg.appendChild(s('text', { x: L, y: Hh / 2, class: 'empty' }, 'no trials yet')); return svg; }
  const vmax = Math.max(...tr.map(x => x.value), t.result?.limit ?? 0) * 1.1 || 1;
  const n = Math.max(tr.length, 6);
  const x = (i: number) => L + (i / (n - 1)) * (W - L - R);
  const y = (v: number) => T + (1 - v / vmax) * (Hh - T - B);
  // grid + axis labels
  for (const f of [0, 0.5, 1]) {
    const v = vmax * f;
    svg.appendChild(s('line', { x1: L, x2: W - R, y1: y(v), y2: y(v), class: 'grid' }));
    svg.appendChild(s('text', { x: L - 5, y: y(v) + 3, 'text-anchor': 'end', class: 'ax' }, num(v)));
  }
  svg.appendChild(s('text', { x: L, y: Hh - 3, class: 'ax' }, 'trial 1'));
  svg.appendChild(s('text', { x: W - R, y: Hh - 3, class: 'ax', 'text-anchor': 'end' }, `${tr.length}`));
  if (t.result) svg.appendChild(s('line', { x1: L, x2: W - R, y1: y(t.result.limit), y2: y(t.result.limit), class: 'lim' }));
  svg.appendChild(s('polyline', { points: tr.map((p, i) => `${x(i)},${y(p.value)}`).join(' '), class: 'trace' }));
  tr.forEach((p, i) => {
    if (p.ok) svg.appendChild(s('circle', { cx: x(i), cy: y(p.value), r: 3.2, class: 'pass' }));
    else svg.appendChild(s('path', { d: `M${x(i) - 3.5} ${y(p.value) - 3.5}l7 7m0 -7l-7 7`, class: 'fail' }));
  });
  return svg;
}

export function calibration(root: HTMLElement) {
  const selected = new Set(Object.keys(LABEL));
  let cal: Cal | null = null;
  let st: Status | null = null;
  const list = h('div', { class: 'tests' });
  const results = h('section', { class: 'panel' });
  const pick = h('div', { class: 'picks' });
  const startB = h('button', { type: 'button', class: 'solid', onClick: () => { confirmBar.hidden = false; } }, 'Start');
  const abortB = h('button', { type: 'button', class: 'danger', onClick: async () => { try { await api('/cal/abort', 'POST'); load(); } catch (e) { toast((e as ApiError).message, true); } } }, 'Abort');
  const note = h('p', { class: 'hint' });
  const confirmBar = h('div', { class: 'confirm', hidden: true },
    h('span', null, 'The axis will travel its full range at increasing speeds. Make sure the carriage path is clear.'),
    h('button', { type: 'button', class: 'solid', onClick: async () => {
      confirmBar.hidden = true;
      try { await api('/cal/start', 'POST', { tests: [...selected] }); toast('Calibration started'); load(); }
      catch (e) { toast((e as ApiError).message, true); }
    } }, 'Run'),
    h('button', { type: 'button', onClick: () => { confirmBar.hidden = true; } }, 'Cancel'));

  root.append(
    h('section', { class: 'panel' }, h('h2', null, 'Tests'), pick, note, h('div', { class: 'row end' }, abortB, startB), confirmBar),
    list, results);

  function draw() {
    const running = !!cal?.running;
    pick.replaceChildren(...Object.keys(LABEL).map(k => h('label', { class: 'chk' },
      h('input', { type: 'checkbox', checked: selected.has(k), disabled: running, onChange: (e: Event) => { (e.target as HTMLInputElement).checked ? selected.add(k) : selected.delete(k); draw(); } }),
      h('span', null, h('b', null, nm(k)), h('small', null, LABEL[k][1])))));
    const homed = !!st?.homed;
    (startB as HTMLButtonElement).disabled = running || !selected.size || !homed;
    (abortB as HTMLButtonElement).disabled = !running;
    note.textContent = running ? 'Running. Abort stops the axis and it will need homing.' : !homed ? 'Home the axis first.' : 'Each test sweeps upward until steps are lost, bisects three times, then sets the working value to 80 % of the limit.';
    if (!cal) return;
    const cw = Math.max(240, Math.floor(root.clientWidth - 38));
    list.replaceChildren(...cal.tests.filter(t => t.status !== 'skipped').map(t => h('section', { class: `panel test ${t.status}` },
      h('h2', null, nm(t.name), h('span', { class: 'tag mono' }, t.status.toUpperCase())),
      h('div', { class: 'meter', role: 'progressbar', 'aria-valuenow': Math.round(t.progress * 100), 'aria-valuemin': 0, 'aria-valuemax': 100 }, h('i', { style: `width:${t.progress * 100}%` })),
      chart(t, cw),
      h('div', { class: 'legend mono' }, h('span', { class: 'k pass' }, 'held'), h('span', { class: 'k fail' }, 'lost steps'), t.result ? h('span', { class: 'k lim' }, 'limit') : null))));
    const done = cal.tests.filter(t => t.result);
    results.hidden = !done.length;
    results.replaceChildren(h('h2', null, 'Results'),
      h('table', { class: 'mono' },
        h('thead', null, h('tr', null, ['Test', 'Limit', 'Working', 'Unit'].map(x => h('th', null, x)))),
        h('tbody', null, done.map(t => h('tr', null,
          h('td', null, nm(t.name)), h('td', null, num(t.result!.limit)), h('td', { class: 'acc' }, num(t.result!.working)), h('td', null, t.result!.unit))))));
  }

  async function load() { try { cal = await api<Cal>('/cal'); draw(); } catch (e) { note.textContent = (e as ApiError).message; } }
  load();
  const offC = onCal(c => { cal = c; draw(); });
  const offS = onStatus(x => { const was = st?.state; st = x; if (cal?.running || was === 'CAL' || x?.state === 'CAL') load(); else draw(); });
  const iv = window.setInterval(() => { if (cal?.running) load(); }, 700);
  return () => { offC(); offS(); clearInterval(iv); };
}
