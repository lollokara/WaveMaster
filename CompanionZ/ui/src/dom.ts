type Child = Node | string | number | false | null | undefined | Child[];
type Props = Record<string, unknown>;

export function h<K extends keyof HTMLElementTagNameMap>(tag: K, props?: Props | null, ...kids: Child[]): HTMLElementTagNameMap[K] {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(props ?? {})) {
    if (v == null || v === false) continue;
    if (k.startsWith('on')) el.addEventListener(k.slice(2).toLowerCase(), v as EventListener);
    else if (k === 'class') el.className = v as string;
    else if (k in el && k !== 'list' && k !== 'type') (el as any)[k] = v;
    else el.setAttribute(k, v === true ? '' : String(v));
  }
  add(el, kids);
  return el;
}
function add(el: Node, kids: Child[]) {
  for (const k of kids) {
    if (Array.isArray(k)) add(el, k);
    else if (k != null && k !== false) el.appendChild(typeof k === 'object' ? k : document.createTextNode(String(k)));
  }
}

const NS = 'http://www.w3.org/2000/svg';
export function s(tag: string, props?: Record<string, string | number> | null, ...kids: (Node | string)[]): SVGElement {
  const el = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(props ?? {})) el.setAttribute(k, String(v));
  for (const k of kids) el.appendChild(typeof k === 'string' ? document.createTextNode(k) : k);
  return el;
}

export const fmt = (n: number, d = 2) => n.toFixed(d);
export const clear = (el: Element) => { while (el.firstChild) el.removeChild(el.firstChild); };

let toastEl: HTMLElement | null = null, toastT = 0;
export function toast(msg: string, bad = false) {
  if (!toastEl) { toastEl = h('div', { class: 'toast', role: 'status', 'aria-live': 'polite' }); document.body.appendChild(toastEl); }
  toastEl.textContent = msg; toastEl.className = 'toast show' + (bad ? ' bad' : '');
  clearTimeout(toastT); toastT = window.setTimeout(() => toastEl!.classList.remove('show'), 3800);
}
