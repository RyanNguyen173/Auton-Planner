/* Shell services shared by both views: theme, hover help, modal dialogs, downloads.
   Views never touch #tip / #modal directly; they go through these functions. */

export const esc = s => String(s).replace(/[&<>"]/g, m => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[m]));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

/* ---------------- Downloads ---------------- */
export function downloadBlob(blob, name) {
  const u = URL.createObjectURL(blob), a = document.createElement('a');
  a.href = u; a.download = name; document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(u), 1000);
}

/* ---------------- Modal ---------------- */
const modalEl = () => document.getElementById('modal');
const modalBox = () => document.getElementById('modalbox');
export const isModalOpen = () => !modalEl().hidden;
/** Query inside the open modal (for buttons a view put there with showModal). */
export const mq = s => modalBox().querySelector(s);
export function showModal(html) {
  modalBox().innerHTML = html; modalEl().hidden = false;
  const cb = mq('#closeM'); if (cb) cb.onclick = closeModal;
}
export function closeModal() { modalEl().hidden = true; }
/** Two-step confirmation for anything destructive: a first popup, then "Are you sure?". */
export function confirmDelete(title, detail, onYes) {
  const step = n => {
    showModal(`<div style="max-width:380px;display:grid;gap:10px"><strong>${n === 1 ? esc(title) : 'Are you sure?'}</strong>
      <p class="hint">${n === 1 ? esc(detail) : "This can't be undone."}</p>
      <div class="flexrow" style="justify-content:flex-end"><button class="btn" id="cfNo">Cancel</button><button class="btn ${n === 1 ? 'primary' : 'primary danger-fill'}" id="cfYes">${n === 1 ? 'Delete' : 'Yes, delete'}</button></div></div>`);
    mq('#cfNo').onclick = closeModal;
    mq('#cfYes').onclick = () => { if (n === 1) step(2); else { closeModal(); onYes(); } };
    mq('#cfNo').focus();
  };
  step(1);
}
export function initModal() {
  modalEl().addEventListener('click', e => { if (e.target.id === 'modal') closeModal(); });
  document.addEventListener('keydown', e => { if (e.key === 'Escape' && isModalOpen()) closeModal(); });
}

/* ---------------- Hover help ----------------
   Elements carry data-tip="key". The text comes from the dictionary registered for the pane the element
   sits in (closest [data-pane]), falling back to the "shell" dictionary. */
const DICTS = {};
export function registerTips(pane, dict) { DICTS[pane] = { ...(DICTS[pane] || {}), ...dict }; }
function tipText(el) {
  const pane = el.closest('[data-pane]')?.dataset.pane || 'shell', key = el.dataset.tip;
  return (DICTS[pane] || {})[key] ?? (DICTS.shell || {})[key];
}
export function initTooltips() {
  const tipEl = document.getElementById('tip'); let timer = 0, on = null;
  const show = el => {
    const html = tipText(el); if (!html) return;
    tipEl.innerHTML = html; tipEl.hidden = false; on = el;
    const r = el.getBoundingClientRect(), tw = tipEl.offsetWidth, th = tipEl.offsetHeight;
    let x = clamp(r.left + r.width / 2 - tw / 2, 12, innerWidth - tw - 12), y = r.bottom + 8;
    if (y + th > innerHeight - 8) y = r.top - th - 8;
    tipEl.style.left = x + 'px'; tipEl.style.top = Math.max(8, y) + 'px';
  };
  const hide = () => { clearTimeout(timer); tipEl.hidden = true; on = null; };
  document.addEventListener('mouseover', e => { const el = e.target.closest('[data-tip]'); if (el === on) return; hide(); if (el) timer = setTimeout(() => show(el), 280); });
  document.addEventListener('focusin', e => { const el = e.target.closest('[data-tip]'); hide(); if (el) show(el); });
  document.addEventListener('focusout', hide); document.addEventListener('scroll', hide, true); document.addEventListener('pointerdown', hide);
}

/* ---------------- Theme ----------------
   No data-theme attribute = follow the OS. The toggle sets data-theme="light"|"dark" and remembers it. */
const THEMEKEY = 'override-planner-theme';
export const isDark = () => { const t = document.documentElement.dataset.theme; return t ? t === 'dark' : matchMedia('(prefers-color-scheme: dark)').matches; };
const SUN = '<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><circle cx="8" cy="8" r="3"/><path d="M8 1.5v1.5M8 13v1.5M1.5 8H3M13 8h1.5M3.4 3.4l1 1M11.6 11.6l1 1M3.4 12.6l1-1M11.6 4.4l1-1"/></svg>';
const MOON = '<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><path d="M13.5 9.5A5.5 5.5 0 0 1 6.5 2.5a5.5 5.5 0 1 0 7 7z"/></svg>';
export function initTheme(btn) {
  try { const t = localStorage.getItem(THEMEKEY); if (t === 'dark' || t === 'light') document.documentElement.dataset.theme = t; } catch (_) {}
  const sync = () => { const d = isDark(); btn.innerHTML = d ? SUN : MOON; btn.setAttribute('aria-label', d ? 'Switch to light mode' : 'Switch to dark mode'); };
  btn.onclick = () => {
    const next = isDark() ? 'light' : 'dark'; document.documentElement.dataset.theme = next;
    try { localStorage.setItem(THEMEKEY, next); } catch (_) {}
    sync(); window.dispatchEvent(new Event('themechange'));
  };
  matchMedia('(prefers-color-scheme: dark)').addEventListener?.('change', () => { sync(); window.dispatchEvent(new Event('themechange')); });
  sync();
}
/** Views call this to redraw canvases (which read CSS variables) when the theme changes. */
export const onThemeChange = fn => window.addEventListener('themechange', fn);
