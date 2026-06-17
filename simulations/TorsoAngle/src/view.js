// view.js — canvas ownership, world->screen transform, and drawing primitives.
// World units are mm with +x right, +y DOWN (matches the physics). The camera
// can pan in x (camX) so the robot stays centred while terrain scrolls.

export const canvas = document.getElementById('legCanvas');
export const ctx = canvas.getContext('2d');

export const PX_PER_N = 3;                 // reaction / weight arrow scale

// Origin sits low in the canvas so the legs hang into the lower two-thirds and
// the torso/hips are not clipped at the top.
export const VIEW = {
    ox: canvas.width / 2,
    oy: canvas.height * 0.62,
    scale: 1.05,
    camX: 0,                               // world x kept at horizontal centre
};

export function setCamera(worldX) { VIEW.camX = worldX; }

export function cv(p) {
    return { x: VIEW.ox + (p.x - VIEW.camX) * VIEW.scale, y: VIEW.oy + p.y * VIEW.scale };
}

export function clear() { ctx.clearRect(0, 0, canvas.width, canvas.height); }

export function line(a, b, color, width, dash = []) {
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    ctx.restore();
}

export function dot(p, r, color) {
    ctx.save(); ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.fill(); ctx.restore();
}

export function arrow(from, to, color, width = 2.5) {
    if (Math.hypot(to.x - from.x, to.y - from.y) < 2) return;
    const a = Math.atan2(to.y - from.y, to.x - from.x);
    const head = 10;
    ctx.save();
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(from.x, from.y); ctx.lineTo(to.x, to.y); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(to.x, to.y);
    ctx.lineTo(to.x - head * Math.cos(a - Math.PI / 6), to.y - head * Math.sin(a - Math.PI / 6));
    ctx.lineTo(to.x - head * Math.cos(a + Math.PI / 6), to.y - head * Math.sin(a + Math.PI / 6));
    ctx.closePath(); ctx.fill(); ctx.restore();
}

export function label(p, text, color, align = 'center') {
    ctx.save(); ctx.fillStyle = color; ctx.font = 'bold 11px monospace';
    ctx.textAlign = align; ctx.textBaseline = 'middle';
    ctx.fillText(text, p.x, p.y); ctx.restore();
}

export function centeredText(text, color) {
    ctx.save();
    ctx.fillStyle = color; ctx.font = '13px monospace'; ctx.textAlign = 'center';
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
    ctx.restore();
}
