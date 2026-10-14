export interface Status { pos: number; state: string; homed: boolean; endstop: boolean; fw: string; rssi: number; s3: boolean; uptime: number }
export interface Item { key: string; label: string; value: number | string | boolean; min?: number; max?: number; step?: number; unit?: string; type: 'number' | 'bool' | 'enum'; options?: (string | { value: string; label?: string })[] }
export interface Group { name: string; items: Item[] }
export interface Trial { value: number; ok: boolean }
export interface CalTest { name: string; status: 'pending' | 'running' | 'ok' | 'failed' | 'skipped'; progress: number; result: null | { limit: number; unit: string; working: number }; trials: Trial[] }
export interface Cal { running: boolean; current?: string; tests: CalTest[] }

export class ApiError extends Error { constructor(msg: string, public key?: string, public code = 0) { super(msg); } }

export async function api<T = any>(path: string, method = 'GET', body?: unknown): Promise<T> {
  let r: Response;
  try {
    r = await fetch('/api' + path, { method, headers: body ? { 'content-type': 'application/json' } : undefined, body: body ? JSON.stringify(body) : undefined });
  } catch { throw new ApiError('No connection to Z-Bro'); }
  const txt = await r.text();
  let j: any = null; try { j = txt ? JSON.parse(txt) : null; } catch { /* non-JSON reply */ }
  if (!r.ok) throw new ApiError(j?.error ?? `HTTP ${r.status}`, j?.key, r.status);
  return j as T;
}

// ---- live status link: WebSocket on the device, polling in dev or while the socket is down ----
type Fn<T> = (v: T) => void;
const statusSubs = new Set<Fn<Status | null>>();
const calSubs = new Set<Fn<Cal>>();
export let status: Status | null = null;

function setStatus(s: Status | null) { status = s; statusSubs.forEach(f => f(s)); }
export const onStatus = (f: Fn<Status | null>) => { statusSubs.add(f); f(status); return () => { statusSubs.delete(f); }; };
export const onCal = (f: Fn<Cal>) => { calSubs.add(f); return () => { calSubs.delete(f); }; };

export async function pollStatus() {
  try { setStatus(await api<Status>('/status')); } catch { setStatus(null); }
}

export function startLink() {
  let ws: WebSocket | null = null;
  let wsOpen = false;
  const connect = () => {
    if (import.meta.env.DEV) return; // the mock server has no WebSocket
    try {
      ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
      ws.onopen = () => { wsOpen = true; };
      ws.onmessage = e => {
        try {
          const m = JSON.parse(e.data);
          if (m.t === 'status') setStatus(m as Status);
          else if (m.t === 'cal') calSubs.forEach(f => f(m as Cal));
        } catch { /* ignore */ }
      };
      ws.onclose = () => { wsOpen = false; setTimeout(connect, 2000); };
    } catch { setTimeout(connect, 2000); }
  };
  connect();
  const loop = async () => {
    if (!wsOpen && !document.hidden) await pollStatus();
    setTimeout(loop, import.meta.env.DEV ? 250 : 1000);
  };
  loop();
}
