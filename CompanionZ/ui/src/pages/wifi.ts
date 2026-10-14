import { h, toast } from '../dom';
import { api, ApiError } from '../api';

export function wifi(root: HTMLElement) {
  const ssid = h('input', { id: 'ssid', type: 'text', autocomplete: 'off', autocapitalize: 'off', spellcheck: false, maxLength: 32 });
  const pass = h('input', { id: 'pass', type: 'password', autocomplete: 'new-password', placeholder: 'Unchanged', maxLength: 63 });
  const err = h('div', { class: 'err', role: 'alert' });
  const state = h('div', { class: 'sub mono' });
  const reboot = h('div', { class: 'confirm', hidden: true },
    h('span', null, 'Restart the controller? The axis must be homed again.'),
    h('button', { type: 'button', class: 'danger', onClick: async () => { try { await api('/reboot', 'POST'); toast('Rebooting. Reconnect in a few seconds.'); } catch (e) { toast((e as ApiError).message, true); } reboot.hidden = true; } }, 'Restart'),
    h('button', { type: 'button', onClick: () => { reboot.hidden = true; } }, 'Cancel'));

  const hub = h('input', { id: 'fleet_hub', type: 'text', autocomplete: 'off', autocapitalize: 'off', spellcheck: false, maxLength: 127, placeholder: 'http://192.168.3.87:8080' });
  const fleetState = h('div', { class: 'sub mono' });
  const fleetErr = h('div', { class: 'err', role: 'alert' });
  const fleet = h('section', { class: 'panel narrow' },
    h('h2', null, 'Fleet'),
    fleetState,
    h('div', { class: 'field' }, h('label', { for: 'fleet_hub' }, 'Fleet hub URL'), h('div', { class: 'ctl' }, hub),
      h('div', { class: 'hint' }, 'Empty: discover the hub via mDNS. "off": disable the agent. Applies after a reboot.')),
    fleetErr,
    h('div', { class: 'row end' },
      h('button', { type: 'button', class: 'solid', onClick: async () => {
        fleetErr.textContent = '';
        try { await api('/wifi', 'PUT', { fleet_hub: hub.value.trim() }); toast('Saved. Reboot to apply.'); }
        catch (e) { fleetErr.textContent = (e as ApiError).message; }
      } }, 'Save')));

  root.append(
    h('section', { class: 'panel narrow' },
      h('h2', null, 'Network'),
      state,
      h('div', { class: 'field' }, h('label', { for: 'ssid' }, 'SSID'), h('div', { class: 'ctl' }, ssid)),
      h('div', { class: 'field' }, h('label', { for: 'pass' }, 'Password'), h('div', { class: 'ctl' }, pass), h('div', { class: 'hint' }, 'Write-only. Leave empty to keep the stored password.')),
      err,
      h('div', { class: 'row end' },
        h('button', { type: 'button', onClick: () => { reboot.hidden = false; } }, 'Reboot'),
        h('button', { type: 'button', class: 'solid', onClick: async () => {
          err.textContent = '';
          try { await api('/wifi', 'PUT', { ssid: ssid.value, pass: pass.value }); pass.value = ''; toast('Saved. Reboot to join the new network.'); }
          catch (e) { err.textContent = (e as ApiError).message; }
        } }, 'Save')),
      reboot),
    fleet);

  api<{ ssid: string; mode?: string; ip?: string; fleet_hub?: string; fleet?: { enabled: boolean; registered: boolean; hb_age: number; hub: string } }>('/wifi').then(r => {
    ssid.value = r.ssid ?? '';
    hub.value = r.fleet_hub ?? '';
    const f = r.fleet;
    fleetState.textContent = !f ? '' : !f.enabled ? 'DISABLED'
      : f.registered ? `REGISTERED  ${f.hub}  last heartbeat ${f.hb_age} s ago` : `NOT REGISTERED  ${f.hub || 'searching'}`;
    state.textContent = [r.mode === 'AP' ? 'SOFT AP' : r.mode === 'STA' ? 'STATION' : '', r.ip].filter(Boolean).join('  ');
  }).catch(e => { err.textContent = (e as ApiError).message; });
  return () => {};
}
