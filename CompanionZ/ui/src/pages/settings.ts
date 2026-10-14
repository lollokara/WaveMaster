import { h, toast, fmt } from '../dom';
import { api, ApiError, Group, Item, status } from '../api';

type V = number | string | boolean;
const optVal = (o: string | { value: string }) => (typeof o === 'string' ? o : o.value);
const optLabel = (o: string | { value: string; label?: string }) => (typeof o === 'string' ? o : o.label ?? o.value);

export function settings(root: HTMLElement) {
  let groups: Group[] = [];
  const orig = new Map<string, V>();
  const cur = new Map<string, V>();
  const errs = new Map<string, string>();
  const rows = new Map<string, { row: HTMLElement; err: HTMLElement }>();
  const body = h('div', { class: 'groups' });
  const readout = h('section', { class: 'panel summary mono', 'aria-live': 'polite' });
  const saveB = h('button', { type: 'button', class: 'solid', onClick: () => save() }, 'Save');
  const resetB = h('button', { type: 'button', onClick: () => { cur.clear(); orig.forEach((v, k) => cur.set(k, v)); errs.clear(); build(); } }, 'Discard');
  const bar = h('div', { class: 'savebar' }, h('span', { class: 'mono count' }), h('div', { class: 'spacer' }), resetB, saveB);
  root.append(readout, body, bar);

  const dirtyKeys = () => [...cur.keys()].filter(k => cur.get(k) !== orig.get(k));
  const num = (k: string) => Number(cur.get(k));

  function refresh() {
    const d = dirtyKeys();
    (bar.firstChild as HTMLElement).textContent = d.length ? `${d.length} unsaved change${d.length > 1 ? 's' : ''}` : 'No changes';
    bar.classList.toggle('on', d.length > 0);
    (saveB as HTMLButtonElement).disabled = !d.length;
    (resetB as HTMLButtonElement).disabled = !d.length;
    rows.forEach((r, k) => { r.row.classList.toggle('dirty', cur.get(k) !== orig.get(k)); r.err.textContent = errs.get(k) ?? ''; r.row.classList.toggle('bad', errs.has(k)); });
    // derived motion constants
    const spm = (num('steps_rev') * num('microstep')) / num('lead_mm');
    const ok = isFinite(spm) && spm > 0;
    const stat = (label: string, v: string, u: string) => h('div', null, h('span', null, label), h('b', null, v, h('small', null, ' ' + u)));
    readout.replaceChildren(
      stat('Steps per mm', ok ? fmt(spm, 2) : '--', 'step/mm'),
      stat('Resolution', ok ? fmt(1000 / spm, 3) : '--', 'um/step'),
      stat('Step rate at max speed', ok ? fmt((spm * num('max_speed')) / 1000, 1) : '--', 'kHz'));
  }

  function field(it: Item): HTMLElement {
    const id = 'f-' + it.key;
    const err = h('div', { class: 'err', id: id + '-e', role: 'alert' });
    let ctl: HTMLElement;
    const set = (v: V) => { cur.set(it.key, v); errs.delete(it.key); refresh(); };
    if (it.type === 'bool') {
      ctl = h('button', { type: 'button', class: 'sw', role: 'switch', id, 'aria-checked': String(!!cur.get(it.key)),
        onClick: (e: Event) => { const n = !cur.get(it.key); (e.currentTarget as HTMLElement).setAttribute('aria-checked', String(n)); set(n); } }, h('i'));
    } else if (it.type === 'enum') {
      ctl = h('select', { id, onChange: (e: Event) => set((e.target as HTMLSelectElement).value) },
        (it.options ?? []).map(o => h('option', { value: optVal(o), selected: String(cur.get(it.key)) === optVal(o) }, optLabel(o))));
    } else {
      const inp = h('input', { id, type: 'number', inputMode: 'decimal', value: String(cur.get(it.key)), step: it.step ?? 'any', min: it.min, max: it.max, 'aria-describedby': id + '-e',
        onInput: (e: Event) => {
          const raw = (e.target as HTMLInputElement).value, v = parseFloat(raw);
          if (raw === '' || isNaN(v)) { cur.set(it.key, NaN); errs.set(it.key, 'Enter a number'); }
          else {
            cur.set(it.key, v); errs.delete(it.key);
            if (it.min != null && v < it.min) errs.set(it.key, `Minimum ${it.min}`);
            else if (it.max != null && v > it.max) errs.set(it.key, `Maximum ${it.max}`);
          }
          refresh();
        } });
      ctl = h('span', { class: 'inp' }, inp, it.unit ? h('em', null, it.unit) : null);
    }
    const range = it.type === 'number' && (it.min != null || it.max != null) ? h('span', { class: 'range mono' }, `${it.min ?? ''} to ${it.max ?? ''}`) : null;
    const extra = it.key === 'max_range'
      ? h('button', { type: 'button', class: 'link', onClick: () => {
          if (!status) return toast('No position available', true);
          const v = Math.round(status.pos * 100) / 100;
          if (v <= 0) return toast('Move the axis away from home first', true);
          cur.set(it.key, v); build(); toast(`Range set to ${v} mm (unsaved)`);
        } }, 'Set from current position') : null;
    const row = h('div', { class: 'field' }, h('label', { for: id }, it.label, h('code', null, it.key)), h('div', { class: 'ctl' }, ctl, range, extra), err);
    rows.set(it.key, { row, err });
    return row;
  }

  function build() {
    rows.clear(); body.replaceChildren();
    for (const g of groups) body.appendChild(h('section', { class: 'panel' }, h('h2', null, g.name), g.items.map(field)));
    refresh();
  }

  async function load() {
    try {
      const r = await api<{ groups: Group[] }>('/settings');
      groups = r.groups; orig.clear(); cur.clear(); errs.clear();
      for (const it of groups.flatMap(g => g.items)) { orig.set(it.key, it.value); cur.set(it.key, it.value); }
      build();
    } catch (e) { body.replaceChildren(h('p', { class: 'hint' }, (e as ApiError).message)); }
  }

  async function save() {
    const d = dirtyKeys();
    if ([...errs.keys()].length) return toast('Fix the highlighted fields first', true);
    const payload: Record<string, V> = {};
    for (const k of d) payload[k] = cur.get(k)!;
    (saveB as HTMLButtonElement).disabled = true;
    try {
      await api('/settings', 'PUT', payload);
      toast('Settings saved');
      await load();
    } catch (e) {
      const ae = e as ApiError;
      if (ae.key && rows.has(ae.key)) {
        errs.set(ae.key, ae.message); refresh();
        rows.get(ae.key)!.row.scrollIntoView({ block: 'center', behavior: 'smooth' });
      }
      toast(ae.key ? `${ae.key}: ${ae.message}` : ae.message, true);
    }
    refresh();
  }

  load();
  const warn = (e: BeforeUnloadEvent) => { if (dirtyKeys().length) e.preventDefault(); };
  window.addEventListener('beforeunload', warn);
  return () => window.removeEventListener('beforeunload', warn);
}
