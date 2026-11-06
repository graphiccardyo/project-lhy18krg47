// 가벼운 캔버스 차트 (외부 라이브러리 없음, 오프라인 동작)
// drawChart(canvas, {
//   x: [...],                                   // 기본 x
//   series: [{ y, x?, color:'--s1', axis:'L'|'R', width, dash, points, label }],
//   xmin, xmax, yL:[min,max]|null, yR:[min,max]|null, xlabel, yLabelL, yLabelR, zeroLine
// })
(function (root) {
  'use strict';

  function niceStep(range, count) {
    const raw = range / Math.max(1, count);
    const p = Math.pow(10, Math.floor(Math.log10(raw)));
    const f = raw / p;
    return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p;
  }

  function ticks(min, max, count) {
    if (!(max > min)) return [min];
    const s = niceStep(max - min, count), out = [];
    for (let v = Math.ceil(min / s) * s; v <= max + s * 1e-9; v += s) out.push(Math.abs(v) < s * 1e-9 ? 0 : v);
    return out;
  }

  function fmt(v, step) {
    const d = step >= 1 ? 0 : Math.min(4, Math.ceil(-Math.log10(step)));
    return v.toFixed(d);
  }

  function range(series, axis, xmin, xmax, fallbackX) {
    let lo = Infinity, hi = -Infinity;
    for (const s of series) {
      if ((s.axis || 'L') !== axis || !s.y) continue;
      const xs = s.x || fallbackX;
      for (let i = 0; i < s.y.length; i++) {
        const xv = xs ? xs[i] : i, v = s.y[i];
        if (xv < xmin || xv > xmax || !Number.isFinite(v)) continue;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    }
    if (!Number.isFinite(lo)) return null;
    if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.08;
    return [lo - pad, hi + pad];
  }

  function drawChart(canvas, o) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(document.documentElement);
    const col = (c) => (c && c.startsWith('--') ? css.getPropertyValue(c).trim() : c) || '#888';
    const series = o.series || [];
    const hasR = series.some((s) => s.axis === 'R');

    let xmin = o.xmin, xmax = o.xmax;
    if (xmin == null || xmax == null) {
      let lo = Infinity, hi = -Infinity;
      for (const s of series) {
        const xs = s.x || o.x;
        if (!xs) continue;
        for (const v of xs) { if (v < lo) lo = v; if (v > hi) hi = v; }
      }
      if (xmin == null) xmin = Number.isFinite(lo) ? lo : 0;
      if (xmax == null) xmax = Number.isFinite(hi) ? hi : 1;
      if (xmax <= xmin) xmax = xmin + 1;
    }
    const yL = o.yL || range(series, 'L', xmin, xmax, o.x) || [-1, 1];
    const yR = hasR ? (o.yR || range(series, 'R', xmin, xmax, o.x) || [0, 1]) : null;

    ctx.font = '11px system-ui, "Segoe UI", "Malgun Gothic", sans-serif';
    const padL = 48, padR = hasR ? 46 : 12, padT = 10, padB = o.xlabel ? 32 : 22;
    const pw = w - padL - padR, ph = h - padT - padB;
    const X = (v) => padL + (v - xmin) / (xmax - xmin) * pw;
    const YL = (v) => padT + (1 - (v - yL[0]) / (yL[1] - yL[0])) * ph;
    const YR = yR ? (v) => padT + (1 - (v - yR[0]) / (yR[1] - yR[0])) * ph : null;

    const grid = col('--grid'), axisCol = col('--axis'), muted = col('--muted');
    ctx.lineWidth = 1;
    const xt = ticks(xmin, xmax, Math.max(2, Math.floor(pw / 80)));
    const xs = xt.length > 1 ? xt[1] - xt[0] : 1;
    ctx.fillStyle = muted; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (const v of xt) {
      const px = X(v);
      ctx.strokeStyle = grid; ctx.beginPath(); ctx.moveTo(px, padT); ctx.lineTo(px, padT + ph); ctx.stroke();
      ctx.fillText(fmt(v, xs), px, padT + ph + 4);
    }
    const ytL = ticks(yL[0], yL[1], Math.max(3, Math.floor(ph / 34)));
    const ysL = ytL.length > 1 ? ytL[1] - ytL[0] : 1;
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (const v of ytL) {
      const py = YL(v);
      ctx.strokeStyle = grid; ctx.beginPath(); ctx.moveTo(padL, py); ctx.lineTo(padL + pw, py); ctx.stroke();
      ctx.fillText(fmt(v, ysL), padL - 5, py);
    }
    if (yR) {
      const ytR = ticks(yR[0], yR[1], Math.max(2, Math.floor(ph / 34)));
      const ysR = ytR.length > 1 ? ytR[1] - ytR[0] : 1;
      ctx.textAlign = 'left';
      for (const v of ytR) ctx.fillText(fmt(v, ysR), padL + pw + 5, YR(v));
    }
    if (o.zeroLine && yL[0] < 0 && yL[1] > 0) {
      ctx.strokeStyle = axisCol; ctx.beginPath(); ctx.moveTo(padL, YL(0)); ctx.lineTo(padL + pw, YL(0)); ctx.stroke();
    }
    ctx.strokeStyle = axisCol; ctx.strokeRect(padL + 0.5, padT + 0.5, pw, ph);
    if (o.xlabel) { ctx.textAlign = 'center'; ctx.textBaseline = 'bottom'; ctx.fillText(o.xlabel, padL + pw / 2, h - 1); }
    ctx.save();
    ctx.textBaseline = 'top'; ctx.textAlign = 'center';
    if (o.yLabelL) { ctx.translate(10, padT + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(o.yLabelL, 0, -9); }
    ctx.restore();
    if (o.yLabelR && yR) {
      ctx.save(); ctx.textBaseline = 'top'; ctx.textAlign = 'center';
      ctx.translate(w - 2, padT + ph / 2); ctx.rotate(Math.PI / 2); ctx.fillText(o.yLabelR, 0, -2); ctx.restore();
    }

    ctx.save();
    ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
    for (const s of series) {
      if (!s.y || !s.y.length) continue;
      const xsrc = s.x || o.x;
      const Y = s.axis === 'R' && YR ? YR : YL;
      ctx.strokeStyle = ctx.fillStyle = col(s.color);
      ctx.lineWidth = s.width || 1.6;
      ctx.setLineDash(s.dash || []);
      if (s.points) {
        for (let i = 0; i < s.y.length; i++) {
          if (!Number.isFinite(s.y[i])) continue;
          ctx.beginPath(); ctx.arc(X(xsrc ? xsrc[i] : i), Y(s.y[i]), s.r || 3.2, 0, 7); ctx.fill();
        }
        if (!s.line) continue;
      }
      ctx.beginPath();
      let pen = false, lastPx = -1e9, lastY = 0;
      for (let i = 0; i < s.y.length; i++) {
        const xv = xsrc ? xsrc[i] : i, v = s.y[i];
        if (!Number.isFinite(v) || xv < xmin - (xmax - xmin) * 0.01 || xv > xmax + (xmax - xmin) * 0.01) { pen = false; continue; }
        const px = X(xv), py = Y(v);
        if (!pen) { ctx.moveTo(px, py); pen = true; lastPx = px; lastY = py; continue; }
        if (s.step) { if (py !== lastY) { ctx.lineTo(px, lastY); ctx.lineTo(px, py); } }
        else if (px - lastPx >= 0.5 || i === s.y.length - 1) ctx.lineTo(px, py);
        else continue;   // 같은 픽셀 열의 점은 건너뜀
        lastPx = px; lastY = py;
      }
      if (s.step && pen) {   // 마지막 값 유지 구간
        let xe = xsrc ? xsrc[s.y.length - 1] : s.y.length - 1;
        ctx.lineTo(Math.min(X(xe), padL + pw), lastY);
      }
      ctx.stroke();
    }
    ctx.restore();
    ctx.setLineDash([]);

    // 범례
    const labeled = series.filter((s) => s.label);
    if (labeled.length) {
      ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      let lx = padL + 8;
      const ly = padT + 9;
      for (const s of labeled) {
        ctx.fillStyle = col(s.color);
        ctx.fillRect(lx, ly - 1.5, 12, 3);
        ctx.fillStyle = muted;
        ctx.fillText(s.label, lx + 16, ly);
        lx += 22 + ctx.measureText(s.label).width + 10;
      }
    }
  }

  root.drawChart = drawChart;
})(window);
