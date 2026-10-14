import { h, s, fmt, toast } from '../dom';
import { api, ApiError, onStatus, Status, Group } from '../api';

const H = 560, TOP = 30, BOT = 30, AX = 92;

function gauge(range: number, flip: boolean) {
  const y = (mm: number) => TOP + (flip ? mm / range : 1 - mm / range) * (H - TOP - BOT);
  const svg = s('svg', { viewBox: `0 0 220 ${H}`, class: 'gauge', role: 'img', 'aria-label': 'Z position scale' });
  const minor = range > 120 ? 5 : 1, major = range > 120 ? 50 : 10;
  svg.appendChild(s('defs', null, s('pattern', { id: 'hatch', width: 5, height: 5, patternUnits: 'userSpaceOnUse', patternTransform: 'rotate(45)' }, s('line', { x1: 0, y1: 0, x2: 0, y2: 5, class: 'hatch' }))));
  const g = s('g', { class: 'ticks' });
  for (let mm = 0; mm <= range + 1e-6; mm += minor) {
    const isMaj = Math.abs(mm % major) < 1e-6, isMid = !isMaj && Math.abs(mm % (major / 2)) < 1e-6;
    const len = isMaj ? 18 : isMid ? 12 : 6;
    g.appendChild(s('line', { x1: AX - len, x2: AX, y1: y(mm), y2: y(mm), class: isMaj ? 'maj' : '' }));
    if (isMaj) g.appendChild(s('text', { x: AX - 24, y: y(mm) + 3.5, 'text-anchor': 'end' }, String(mm)));
  }
  svg.appendChild(g);
  // axis + travelled strip
  svg.appendChild(s('line', { x1: AX, x2: AX, y1: y(0), y2: y(range), class: 'axis' }));
  const strip = s('line', { x1: AX + 5, x2: AX + 5, y1: y(0), y2: y(0), class: 'strip' });
  svg.appendChild(strip);
  // endstop block at home end, outside the range
  const eyH = flip ? y(0) - 14 : y(0) + 4;
  const es = s('g', { class: 'endstop' },
    s('rect', { x: AX - 6, y: eyH, width: 110, height: 10, fill: 'url(#hatch)' }),
    s('text', { x: AX + 8, y: flip ? eyH - 5 : eyH + 22 }, 'ENDSTOP / HOME 0'));
  svg.appendChild(es);
  // range limit
  const ly = y(range);
  svg.appendChild(s('g', { class: 'limit' },
    s('line', { x1: AX - 6, x2: AX + 110, y1: ly, y2: ly }),
    s('text', { x: AX + 8, y: flip ? ly + 16 : ly - 6 }, `LIMIT ${range}`)));
  // carriage
  const car = s('g', { class: 'carriage' },
    s('line', { x1: AX - 10, x2: AX + 112, y1: 0, y2: 0, class: 'hair' }),
    s('rect', { x: AX + 14, y: -9, width: 70, height: 18, rx: 1 }),
    s('path', { d: `M${AX} 0 l12 -6 v12z`, class: 'ptr' }),
    s('text', { x: AX + 49, y: 4, 'text-anchor': 'middle', class: 'lens' }, 'LENS'));
  svg.appendChild(car);
  return {
    svg,
    set(pos: number, endstop: boolean) {
      const p = Math.min(Math.max(pos, -range * 0.02), range * 1.02);
      car.style.transform = `translateY(${y(p)}px)`;
      strip.setAttribute('y2', String(y(Math.max(0, Math.min(range, pos)))));
      es.classList.toggle('hit', endstop);
    },
  };
}

export function dashboard(root: HTMLElement) {
  let g: ReturnType<typeof gauge> | null = null;
  let range = 50, flip = false, maxSpeed = 20;
  const gaugeBox = h('div', { class: 'gauge-box' });
  const pos = h('div', { class: 'readout mono' });
  const sub = h('div', { class: 'sub mono' });
  const steps = [0.01, 0.1, 1, 10];
  let step = 1;
  const stepBtns = steps.map(v => h('button', { type: 'button', role: 'radio', onClick: () => { step = v; refreshAll(); } }, String(v)));
  const refreshAll = () => {
    stepBtns.forEach((b, i) => b.setAttribute('aria-checked', String(steps[i] === step)));
    // "+" moves away from home: home sits at the bottom unless homing toward max
    plusBtn.replaceChildren(chev(!flip), `+ ${step} mm`);
    minusBtn.replaceChildren(chev(flip), `\u2212 ${step} mm`);
  };

  const target = h('input', { type: 'number', inputMode: 'decimal', step: '0.01', min: '0', id: 'tgt', placeholder: '0.00' });
  const speed = h('input', { type: 'number', inputMode: 'decimal', step: '0.5', min: '0.1', id: 'spd' });

  const act = (fn: () => Promise<unknown>, ok?: string) => async () => {
    try { await fn(); if (ok) toast(ok); } catch (e) { toast((e as ApiError).message, true); }
  };
  const f = () => { const v = parseFloat(speed.value); return v > 0 ? v : undefined; };
  const go = act(() => {
    const z = parseFloat(target.value);
    if (isNaN(z)) throw new ApiError('Enter a target position');
    return api('/move', 'POST', { z, f: f() });
  });
  const jog = (dir: 1 | -1) => act(() => api('/jog', 'POST', { dz: +(dir * step).toFixed(3), f: f() }));
  const chev = (up: boolean) => s('svg', { viewBox: '0 0 12 12', width: 12, height: 12, 'aria-hidden': 'true' },
    s('path', { d: up ? 'M1 8l5-5 5 5' : 'M1 4l5 5 5-5', fill: 'none', stroke: 'currentColor', 'stroke-width': 1.6 }));
  const plusBtn = h('button', { type: 'button', class: 'jog', onClick: jog(1) });
  const minusBtn = h('button', { type: 'button', class: 'jog', onClick: jog(-1) });
  refreshAll();

  const homeB = h('button', { type: 'button', class: 'solid', onClick: act(() => api('/home', 'POST')) }, 'Home');
  const stopB = h('button', { type: 'button', onClick: act(() => api('/stop', 'POST')) }, 'Stop');
  const estopB = h('button', { type: 'button', class: 'danger', onClick: act(() => api('/estop', 'POST'), 'Emergency stop sent. Axis needs homing.') }, 'E-Stop');

  root.append(
    h('section', { class: 'panel gauge-panel' },
      h('h2', null, 'Position'),
      h('div', { class: 'gauge-wrap' }, gaugeBox, h('div', { class: 'reads' }, pos, sub))),
    h('div', { class: 'col' },
      h('section', { class: 'panel' },
        h('h2', null, 'Axis'),
        h('div', { class: 'row three' }, homeB, stopB, estopB),
        h('p', { class: 'hint' }, 'Escape sends Stop. E-Stop halts immediately and the axis must be homed again.')),
      h('section', { class: 'panel' },
        h('h2', null, 'Move to'),
        h('div', { class: 'row' },
          h('label', { for: 'tgt' }, 'Absolute', h('span', { class: 'u' }, h('span', { class: 'inp' }, target, h('em', null, 'mm')))),
          h('label', { for: 'spd' }, 'Speed', h('span', { class: 'u' }, h('span', { class: 'inp' }, speed, h('em', null, 'mm/s')))),
          h('button', { type: 'button', class: 'solid go', onClick: go }, 'Go'))),
      h('section', { class: 'panel' },
        h('h2', null, 'Jog'),
        h('div', { class: 'seg', role: 'radiogroup', 'aria-label': 'Jog step in millimetres' }, stepBtns, h('em', null, 'mm')),
        h('div', { class: 'row two' }, plusBtn, minusBtn))));

  const onKey = (e: KeyboardEvent) => { if (e.key === 'Escape') void api('/stop', 'POST').catch(() => {}); };
  window.addEventListener('keydown', onKey);

  const build = () => {
    gaugeBox.replaceChildren();
    g = gauge(range, flip);
    gaugeBox.appendChild(g.svg);
    refreshAll();
  };
  let last: Status | null = null;
  const render = (st: Status | null) => {
    last = st;
    if (!g) return;
    if (!st) { pos.textContent = '---.---'; sub.textContent = 'no link'; pos.classList.add('dim'); return; }
    pos.classList.remove('dim');
    g.set(st.pos, st.endstop);
    pos.replaceChildren(fmt(st.pos, 3), h('small', null, ' mm'));
    sub.textContent = `${st.homed ? 'HOMED' : 'NOT HOMED'}  ${st.endstop ? 'ENDSTOP ON' : 'endstop off'}`;
    sub.classList.toggle('warn', !st.homed);
    root.dataset.state = st.state;
  };

  // range and home side come from the settings tree
  api<{ groups: Group[] }>('/settings').then(r => {
    for (const it of r.groups.flatMap(x => x.items)) {
      if (it.key === 'max_range') range = Number(it.value);
      if (it.key === 'home_to_max') flip = !!it.value;
      if (it.key === 'max_speed') maxSpeed = Number(it.value);
    }
    speed.placeholder = String(maxSpeed);
  }).catch(() => {}).finally(() => { build(); render(last); });

  const off = onStatus(render);
  return () => { off(); window.removeEventListener('keydown', onKey); };
}
