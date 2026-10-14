// Fake Z-Bro for `npm run dev`: status, motion, settings and a calibration run.
// Only active under the dev server; never part of the build.
import type { Plugin } from 'vite';
import type { IncomingMessage, ServerResponse } from 'http';

type Item = { key: string; label: string; value: number | string | boolean; min?: number; max?: number; step?: number; unit?: string; type: 'number' | 'bool' | 'enum'; options?: string[] };

const groups: { name: string; items: Item[] }[] = [
  { name: 'Mechanics', items: [
    { key: 'steps_rev', label: 'Motor steps per revolution', value: 200, min: 24, max: 400, step: 1, unit: 'steps', type: 'number' },
    { key: 'lead_mm', label: 'Leadscrew lead', value: 8, min: 0.5, max: 40, step: 0.1, unit: 'mm/rev', type: 'number' },
    { key: 'microstep', label: 'Microstepping', value: '16', type: 'enum', options: ['1', '2', '4', '8', '16'] },
    { key: 'dir_invert', label: 'Invert direction', value: false, type: 'bool' },
  ] },
  { name: 'Travel', items: [
    { key: 'max_range', label: 'Usable range', value: 50, min: 1, max: 300, step: 0.1, unit: 'mm', type: 'number' },
    { key: 'home_to_max', label: 'Home toward maximum', value: true, type: 'bool' },
    { key: 'allow_unhomed', label: 'Allow moves while unhomed', value: false, type: 'bool' },
  ] },
  { name: 'Motion', items: [
    { key: 'max_speed', label: 'Maximum speed', value: 20, min: 0.1, max: 60, step: 0.1, unit: 'mm/s', type: 'number' },
    { key: 'accel', label: 'Acceleration', value: 100, min: 1, max: 2000, step: 1, unit: 'mm/s2', type: 'number' },
    { key: 'decel', label: 'Deceleration', value: 100, min: 1, max: 2000, step: 1, unit: 'mm/s2', type: 'number' },
  ] },
  { name: 'Homing and endstop', items: [
    { key: 'es_invert', label: 'Endstop is normally open', value: false, type: 'bool' },
    { key: 'home_fast', label: 'Homing approach speed', value: 10, min: 0.1, max: 40, step: 0.1, unit: 'mm/s', type: 'number' },
    { key: 'home_slow', label: 'Homing final speed', value: 1, min: 0.1, max: 5, step: 0.1, unit: 'mm/s', type: 'number' },
    { key: 'home_backoff', label: 'Back-off distance', value: 2, min: 0.1, max: 10, step: 0.1, unit: 'mm', type: 'number' },
    { key: 'es_debounce_us', label: 'Endstop debounce', value: 2000, min: 0, max: 20000, step: 100, unit: 'us', type: 'number' },
  ] },
  { name: 'Driver and reporting', items: [
    { key: 'pulse_us', label: 'STEP pulse width', value: 2, min: 1, max: 20, step: 1, unit: 'us', type: 'number' },
    { key: 'idle_off_s', label: 'Disable motor after idle', value: 5, min: 0, max: 600, step: 1, unit: 's', type: 'number' },
    { key: 'sleep_idle', label: 'Sleep driver when idle', value: true, type: 'bool' },
    { key: 'rpt_hz', label: 'Position report rate', value: 10, min: 1, max: 50, step: 1, unit: 'Hz', type: 'number' },
  ] },
];
const find = (k: string) => groups.flatMap(g => g.items).find(i => i.key === k);
const val = (k: string) => Number(find(k)!.value);

const st = { pos: 23.4, target: 23.4, vel: 0, dir: 1, speed: 20, state: 'IDLE', homed: false, homing: false, uptime: 0 };

// calibration model: hidden true limits drive pass/fail
type Trial = { value: number; ok: boolean };
type Test = { name: string; status: string; progress: number; result: null | { limit: number; unit: string; working: number }; trials: Trial[]; plan: Trial[]; unit: string };
const hidden: Record<string, { start: number; limit: number; unit: string }> = {
  endstop: { start: 0, limit: 0, unit: 'um' },
  speed: { start: 5, limit: 34, unit: 'mm/s' },
  accel: { start: 50, limit: 420, unit: 'mm/s2' },
  decel: { start: 50, limit: 510, unit: 'mm/s2' },
  homing: { start: 4, limit: 17.5, unit: 'mm/s' },
};
const cal = { running: false, current: '' as string, tests: [] as Test[], queue: [] as string[], tick: 0 };
const resetCal = () => {
  cal.tests = Object.keys(hidden).map(name => ({ name, status: 'pending', progress: 0, result: null, trials: [], plan: [], unit: hidden[name].unit }));
};
resetCal();

function plan(name: string): Trial[] {
  const h = hidden[name];
  if (name === 'endstop') return Array.from({ length: 10 }, () => ({ value: Math.round(8 + Math.random() * 9), ok: true }));
  const out: Trial[] = [];
  let v = h.start, lo = 0, hi = 0;
  for (;;) {
    const ok = v <= h.limit;
    out.push({ value: +v.toFixed(1), ok });
    if (!ok) { hi = v; break; }
    lo = v; v *= 1.5;
  }
  for (let i = 0; i < 3; i++) {
    const m = (lo + hi) / 2, ok = m <= h.limit;
    out.push({ value: +m.toFixed(1), ok });
    if (ok) lo = m; else hi = m;
  }
  return out;
}

function tickMotion(dt: number) {
  const accel = val('accel'), decel = val('decel'), max = val('max_speed');
  const prev = st.pos;
  if (st.state === 'MOVING' || st.state === 'HOMING') {
    const vmax = st.state === 'HOMING' ? val('home_fast') : Math.min(st.speed, max);
    const d = st.target - st.pos;
    st.dir = Math.sign(d) || st.dir;
    const brake = (st.vel * st.vel) / (2 * decel);
    if (Math.abs(d) <= brake + 0.0005 && st.vel > 0) st.vel = Math.max(st.vel - decel * dt, 0.2);
    else st.vel = Math.min(st.vel + accel * dt, vmax);
    const step = st.vel * dt;
    if (Math.abs(d) <= step) {
      st.pos = st.target; st.vel = 0;
      if (st.state === 'HOMING') st.homed = true;
      st.state = 'IDLE';
    } else st.pos += Math.sign(d) * step;
  }
  void prev;
  // simulated calibration: one trial per ~0.55 s
  if (cal.running && st.state === 'CAL') {
    cal.tick += dt;
    if (cal.tick > 0.55) {
      cal.tick = 0;
      let t = cal.tests.find(x => x.name === cal.current);
      if (!t || t.status === 'ok') {
        const next = cal.queue.shift();
        if (!next) { cal.running = false; cal.current = ''; st.state = 'IDLE'; return; }
        cal.current = next; t = cal.tests.find(x => x.name === next)!;
        t.status = 'running'; t.plan = plan(next); t.trials = [];
      }
      const n = t.trials.length;
      if (n < t.plan.length) {
        t.trials.push(t.plan[n]);
        t.progress = t.trials.length / t.plan.length;
      } else {
        t.status = 'ok'; t.progress = 1;
        const h = hidden[t.name];
        if (t.name === 'endstop') {
          const m = Math.max(...t.trials.map(x => x.value));
          t.result = { limit: m, unit: 'um', working: m };
        } else {
          const okv = t.trials.filter(x => x.ok).map(x => x.value);
          const limit = Math.max(...okv);
          t.result = { limit, unit: h.unit, working: +(limit * 0.8).toFixed(1) };
        }
        cal.current = '';
        if (!cal.queue.length) { cal.running = false; st.state = 'IDLE'; st.homed = false; }
      }
    }
  }
}

let last = Date.now();
function body(req: IncomingMessage): Promise<any> {
  return new Promise(res => {
    let s = ''; req.on('data', c => (s += c)); req.on('end', () => { try { res(s ? JSON.parse(s) : {}); } catch { res({}); } });
  });
}
const send = (res: ServerResponse, code: number, o: unknown) => {
  res.statusCode = code; res.setHeader('content-type', 'application/json'); res.end(JSON.stringify(o));
};

export function mockZbro(): Plugin {
  return {
    name: 'zbro-mock',
    apply: 'serve',
    configureServer(server) {
      const iv = setInterval(() => {
        const now = Date.now(); const dt = Math.min((now - last) / 1000, 0.2); last = now;
        st.uptime += dt; tickMotion(dt);
      }, 40);
      server.httpServer?.on('close', () => clearInterval(iv));

      server.middlewares.use('/api', async (req, res) => {
        const path = (req.url ?? '/').split('?')[0];
        const m = req.method;
        await new Promise(r => setTimeout(r, 40 + Math.random() * 60)); // network-ish latency
        const busy = () => st.state !== 'IDLE';
        const range = val('max_range');
        const homeAtMax = !!find('home_to_max')!.value;
        void homeAtMax;

        if (path === '/status' && m === 'GET')
          return send(res, 200, { pos: +st.pos.toFixed(3), state: st.state, homed: st.homed, endstop: st.pos <= 0.001 && st.homed || st.pos < -0.001, fw: '0.4.2-mock', rssi: -58, s3: true, uptime: Math.round(st.uptime) });
        if (path === '/settings' && m === 'GET') return send(res, 200, { groups });
        if (path === '/settings' && m === 'PUT') {
          const b = await body(req);
          for (const [k, v] of Object.entries(b)) {
            const it = find(k);
            if (!it) return send(res, 400, { error: 'unknown setting', key: k });
            if (it.type === 'number') {
              if (typeof v !== 'number' || isNaN(v)) return send(res, 400, { error: 'not a number', key: k });
              if (it.min !== undefined && v < it.min) return send(res, 400, { error: `below minimum ${it.min}`, key: k });
              if (it.max !== undefined && v > it.max) return send(res, 400, { error: `above maximum ${it.max}`, key: k });
              const lim = k === 'max_speed' ? cal.tests[1].result?.limit : undefined;
              if (lim && v > lim) return send(res, 400, { error: `exceeds calibrated limit ${lim}`, key: k });
            }
          }
          for (const [k, v] of Object.entries(b)) find(k)!.value = v as never;
          return send(res, 200, { ok: true });
        }
        if (path === '/move' && m === 'POST') {
          const b = await body(req);
          if (busy()) return send(res, 409, { error: 'busy (1)' });
          if (!st.homed && !find('allow_unhomed')!.value) return send(res, 409, { error: 'not homed (2)' });
          if (b.z < 0 || b.z > range) return send(res, 400, { error: `out of range 0..${range} (3)` });
          st.target = b.z; st.speed = b.f || val('max_speed'); st.state = 'MOVING'; return send(res, 200, { ok: true });
        }
        if (path === '/jog' && m === 'POST') {
          const b = await body(req);
          if (busy()) return send(res, 409, { error: 'busy (1)' });
          if (!st.homed && !find('allow_unhomed')!.value) return send(res, 409, { error: 'not homed (2)' });
          const z = st.pos + b.dz;
          if (z < 0 || z > range) return send(res, 400, { error: `out of range 0..${range} (3)` });
          st.target = z; st.speed = b.f || val('max_speed'); st.state = 'MOVING'; return send(res, 200, { ok: true });
        }
        if (path === '/home' && m === 'POST') {
          if (busy()) return send(res, 409, { error: 'busy (1)' });
          st.target = 0; st.state = 'HOMING'; return send(res, 200, { ok: true });
        }
        if (path === '/stop' && m === 'POST') {
          if (st.state === 'MOVING' || st.state === 'HOMING') {
            st.target = st.pos + Math.sign(st.target - st.pos) * (st.vel * st.vel) / (2 * val('decel'));
            st.state = 'MOVING'; if (st.homed === false && st.target === st.pos) st.state = 'IDLE';
          }
          return send(res, 200, { ok: true });
        }
        if (path === '/estop' && m === 'POST') { st.state = 'IDLE'; st.vel = 0; st.target = st.pos; st.homed = false; cal.running = false; return send(res, 200, { ok: true }); }
        if (path === '/cal' && m === 'GET')
          return send(res, 200, { running: cal.running, current: cal.current, tests: cal.tests.map(({ plan: _p, ...t }) => t) });
        if (path === '/cal/start' && m === 'POST') {
          const b = await body(req);
          if (!st.homed) return send(res, 409, { error: 'not homed (2)' });
          if (busy()) return send(res, 409, { error: 'busy (1)' });
          resetCal();
          const want: string[] = b.tests?.length ? b.tests : Object.keys(hidden);
          cal.tests.forEach(t => { if (!want.includes(t.name)) t.status = 'skipped'; });
          cal.queue = want.filter(w => hidden[w]); cal.current = ''; cal.running = true; st.state = 'CAL';
          return send(res, 200, { ok: true });
        }
        if (path === '/cal/abort' && m === 'POST') {
          cal.running = false; cal.queue = []; st.state = 'IDLE'; st.homed = false;
          cal.tests.forEach(t => { if (t.status === 'running') t.status = 'failed'; if (t.status === 'pending') t.status = 'skipped'; });
          return send(res, 200, { ok: true });
        }
        if (path === '/wifi' && m === 'GET') return send(res, 200, { ssid: 'workshop-2g', mode: 'STA', ip: '192.168.1.57', fleet_hub: 'http://192.168.3.87:8080', fleet: { enabled: true, registered: true, hb_age: 12, hub: 'http://192.168.3.87:8080' } });
        if (path === '/wifi' && m === 'PUT') {
          const b = await body(req);
          if (b.fleet_hub !== undefined && !b.ssid) return send(res, 200, { ok: true, reboot_needed: true });
          if (!b.ssid) return send(res, 400, { error: 'SSID required', key: 'ssid' });
          return send(res, 200, { ok: true });
        }
        if (path === '/reboot' && m === 'POST') return send(res, 200, { ok: true });
        send(res, 404, { error: 'not found' });
      });
    },
  };
}
