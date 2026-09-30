// Interactive 2D version of the collision objective of the paper (Eqs. 1-3, Fig. 2). Two boxes can be
// dragged (inside) and rotated (round handle), and clicking runs one of the optimizers from that seed.
(function () {
    'use strict';

    var canvas = document.getElementById('objective-canvas');
    if (!canvas)
        return;
    var ctx = canvas.getContext('2d');

    // World window, similar to Fig. 2
    var X0 = -120, X1 = 120, Y0 = -90, Y1 = 90;
    var GW = 200, GH = 150; // Resolution of the sampled objective
    var TAU = 0.5;          // Optimizer tolerance in these units
    var MAX_ITERATIONS = 50;

    // The configuration of Fig. 2, with the cached point at the deepest point of the overlap
    var defaults = function () {
        return {
            boxes: [
                { cx: -16, cy: -3, hx: 72, hy: 30, angle: 24 * Math.PI / 180, color: '#2f6db5', dashed: false },
                { cx: 16, cy: 3, hx: 72, hy: 30, angle: -26 * Math.PI / 180, color: '#5e9c3f', dashed: true }
            ],
            points: [{ x: 7.2, y: 7.4 }]
        };
    };

    var state = defaults();
    var settings = { softmax: true, epsilon: 100, alpha: 0.43, optimizer: 'nm', click: 'optimize' };
    var field = new Float32Array(GW * GH);
    var minimum = { x: 0, y: 0, value: 0 };
    var trace = null, traceFrame = 0, traceTimer = null;
    var drag = null;

    // Signed distance of a rotated box and its gradient
    function sdBox(b, x, y) {
        var c = Math.cos(b.angle), s = Math.sin(b.angle);
        var dx = x - b.cx, dy = y - b.cy;
        var lx = c * dx + s * dy, ly = -s * dx + c * dy;
        var qx = Math.abs(lx) - b.hx, qy = Math.abs(ly) - b.hy;
        var ox = Math.max(qx, 0), oy = Math.max(qy, 0);
        var outside = Math.hypot(ox, oy);
        var sx = lx < 0 ? -1 : 1, sy = ly < 0 ? -1 : 1, gx, gy;
        if (outside > 0) { gx = sx * ox / outside; gy = sy * oy / outside; }
        else if (qx > qy) { gx = sx; gy = 0; }
        else { gx = 0; gy = sy; }
        return [outside + Math.min(Math.max(qx, qy), 0), c * gx - s * gy, s * gx + c * gy];
    }

    // Collision objective g(x): softmax (Eq. 2) or hardmax (Eq. 1) of the two fields, minus the
    // repulsion from the cached points (Eq. 3). Returns [value, dg/dx, dg/dy].
    function objective(x, y) {
        var a = sdBox(state.boxes[0], x, y), b = sdBox(state.boxes[1], x, y);
        var v, gx, gy;
        if (settings.softmax) {
            var r = Math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + settings.epsilon);
            var t = r > 0 ? (a[0] - b[0]) / r : 0;
            v = 0.5 * (a[0] + b[0] + r);
            gx = 0.5 * (1 + t) * a[1] + 0.5 * (1 - t) * b[1];
            gy = 0.5 * (1 + t) * a[2] + 0.5 * (1 - t) * b[2];
        } else if (a[0] > b[0]) {
            v = a[0]; gx = a[1]; gy = a[2];
        } else {
            v = b[0]; gx = b[1]; gy = b[2];
        }
        for (var i = 0; i < state.points.length; i++) {
            var px = x - state.points[i].x, py = y - state.points[i].y, l = Math.hypot(px, py);
            v -= settings.alpha * l;
            if (l > 1e-9) { gx -= settings.alpha * px / l; gy -= settings.alpha * py / l; }
        }
        return [v, gx, gy];
    }

    // Search domain V: the overlap of the bounding boxes (Sec. 3.1)
    function boxBounds(b) {
        var c = Math.abs(Math.cos(b.angle)), s = Math.abs(Math.sin(b.angle));
        var ex = c * b.hx + s * b.hy, ey = s * b.hx + c * b.hy;
        return { x0: b.cx - ex, x1: b.cx + ex, y0: b.cy - ey, y1: b.cy + ey };
    }
    function searchDomain() {
        var a = boxBounds(state.boxes[0]), b = boxBounds(state.boxes[1]);
        var v = { x0: Math.max(a.x0, b.x0), x1: Math.min(a.x1, b.x1), y0: Math.max(a.y0, b.y0), y1: Math.min(a.y1, b.y1) };
        if (v.x0 > v.x1 || v.y0 > v.y1)
            v = { x0: Math.min(a.x0, b.x0), x1: Math.max(a.x1, b.x1), y0: Math.min(a.y0, b.y0), y1: Math.max(a.y1, b.y1) };
        return v;
    }

    // The three optimizers of the paper, recording every iteration for the animation

    // Projection onto the search domain V
    function project(V, p) {
        return [Math.min(Math.max(p[0], V.x0), V.x1), Math.min(Math.max(p[1], V.y0), V.y1)];
    }

    function nelderMead(x, y) {
        var V = searchDomain(), evaluations = 0;
        var f = function (p) { evaluations++; return objective(p[0], p[1])[0]; };
        var sx = Math.max(0.1 * (V.x1 - V.x0), TAU), sy = Math.max(0.1 * (V.y1 - V.y0), TAU);
        var p = project(V, [x, y]);
        var v = [p, project(V, [p[0] + (p[0] + sx <= V.x1 ? sx : -sx), p[1]]), project(V, [p[0], p[1] + (p[1] + sy <= V.y1 ? sy : -sy)])];
        var fv = v.map(f), frames = [], it = 0;
        for (; it < MAX_ITERATIONS; it++) {
            var order = [0, 1, 2].sort(function (i, j) { return fv[i] - fv[j]; });
            v = order.map(function (i) { return v[i]; });
            fv = order.map(function (i) { return fv[i]; });
            frames.push({ simplex: v.map(function (p) { return p.slice(); }) });
            var extent = Math.max(Math.hypot(v[1][0] - v[0][0], v[1][1] - v[0][1]), Math.hypot(v[2][0] - v[0][0], v[2][1] - v[0][1]));
            if (extent < TAU || fv[2] - fv[0] < 0.05 * TAU)
                break;
            var c = [(v[0][0] + v[1][0]) / 2, (v[0][1] + v[1][1]) / 2];
            var r = project(V, [2 * c[0] - v[2][0], 2 * c[1] - v[2][1]]), fr = f(r);
            if (fr < fv[0]) {
                var e = project(V, [3 * c[0] - 2 * v[2][0], 3 * c[1] - 2 * v[2][1]]), fe = f(e);
                if (fe < fr) { v[2] = e; fv[2] = fe; } else { v[2] = r; fv[2] = fr; }
            } else if (fr < fv[1]) {
                v[2] = r; fv[2] = fr;
            } else {
                var outside = fr < fv[2], w = outside ? r : v[2];
                var k = [(c[0] + w[0]) / 2, (c[1] + w[1]) / 2], fk = f(k);
                if (fk < Math.min(fr, fv[2])) { v[2] = k; fv[2] = fk; }
                else {
                    for (var i = 1; i < 3; i++) {
                        v[i] = [(v[0][0] + v[i][0]) / 2, (v[0][1] + v[i][1]) / 2];
                        fv[i] = f(v[i]);
                    }
                }
            }
        }
        var best = fv.indexOf(Math.min.apply(null, fv));
        return { frames: frames, x: v[best], iterations: it, evaluations: evaluations };
    }

    function ellipsoidMethod(x, y) {
        // Deep cut ellipsoid method, starting from an ellipse centered at the seed that encloses V
        var V = searchDomain(), n = 2, evaluations = 0;
        var ex = (V.x1 - V.x0) / 2 + Math.abs(x - (V.x0 + V.x1) / 2), ey = (V.y1 - V.y0) / 2 + Math.abs(y - (V.y0 + V.y1) / 2);
        var E = [n * ex * ex, 0, 0, n * ey * ey]; // Row major 2x2
        var p = [x, y], best = project(V, p), bestValue = Infinity, frames = [], it = 0;
        for (; it < MAX_ITERATIONS; it++) {
            frames.push({ center: p.slice(), E: E.slice() });
            // Feasibility cut through the most violated face of V, otherwise a cut along the gradient
            var violations = [V.x0 - p[0], p[0] - V.x1, V.y0 - p[1], p[1] - V.y1];
            var worst = violations.indexOf(Math.max.apply(null, violations)), violation = violations[worst], g;
            if (violation > 0) {
                g = [0, 0, 0];
                g[1 + (worst >> 1)] = worst & 1 ? 1 : -1;
            } else {
                violation = 0;
                g = objective(p[0], p[1]); evaluations++;
                if (g[0] < bestValue) { bestValue = g[0]; best = p.slice(); }
                if (Math.hypot(g[1], g[2]) < 1e-3) break;
            }
            var Eg = [E[0] * g[1] + E[1] * g[2], E[2] * g[1] + E[3] * g[2]];
            var gEg = g[1] * Eg[0] + g[2] * Eg[1];
            if (!(gEg > 1e-12)) break;
            var s = Math.sqrt(gEg), a = Math.min((violation > 0 ? violation : g[0] - bestValue) / s, 0.5);
            var step = (1 + n * a) / ((n + 1) * s);
            p = [p[0] - step * Eg[0], p[1] - step * Eg[1]];
            var k = 2 * (1 + n * a) / ((n + 1) * (1 + a) * gEg), scale = n * n * (1 - a * a) / (n * n - 1);
            E = [scale * (E[0] - k * Eg[0] * Eg[0]), scale * (E[1] - k * Eg[0] * Eg[1]),
                 scale * (E[2] - k * Eg[1] * Eg[0]), scale * (E[3] - k * Eg[1] * Eg[1])];
            if (step * Math.hypot(Eg[0], Eg[1]) < TAU) break;
        }
        return { frames: frames, x: best, iterations: it, evaluations: evaluations };
    }

    function gradientDescent(x, y) {
        // Stochastic projected gradient descent with a decaying step size and gradient noise
        var V = searchDomain(), p = [x, y], best = p.slice(), bestValue = Infinity, frames = [], it = 0, evaluations = 0;
        for (; it < MAX_ITERATIONS; it++) {
            var g = objective(p[0], p[1]); evaluations++;
            frames.push({ point: p.slice() });
            if (g[0] < bestValue) { bestValue = g[0]; best = p.slice(); }
            var gl = Math.hypot(g[1], g[2]);
            if (gl < 1e-3) break;
            var decay = 1 / Math.sqrt(it + 1), angle = Math.random() * 2 * Math.PI, noise = 0.25 * decay * Math.max(gl, 1);
            var next = [p[0] - 12 * decay * (g[1] + noise * Math.cos(angle)), p[1] - 12 * decay * (g[2] + noise * Math.sin(angle))];
            next = [Math.min(Math.max(next[0], V.x0), V.x1), Math.min(Math.max(next[1], V.y0), V.y1)];
            var moved = Math.hypot(next[0] - p[0], next[1] - p[1]);
            p = next;
            if (moved < TAU) break;
        }
        return { frames: frames, x: best, iterations: it, evaluations: evaluations };
    }

    // Coordinate conversions

    function toCanvas(x, y) {
        return [(x - X0) / (X1 - X0) * canvas.width, (Y1 - y) / (Y1 - Y0) * canvas.height];
    }
    function toWorld(px, py) {
        var rect = canvas.getBoundingClientRect();
        return [X0 + (px - rect.left) / rect.width * (X1 - X0), Y1 - (py - rect.top) / rect.height * (Y1 - Y0)];
    }
    function pixelsToWorld(pixels) {
        return pixels * (X1 - X0) / canvas.getBoundingClientRect().width;
    }

    // Sampling and drawing

    function sampleField() {
        minimum.value = Infinity;
        for (var j = 0; j < GH; j++) {
            for (var i = 0; i < GW; i++) {
                var x = X0 + (i + 0.5) / GW * (X1 - X0), y = Y1 - (j + 0.5) / GH * (Y1 - Y0);
                var v = objective(x, y)[0];
                field[j * GW + i] = v;
                if (v < minimum.value) { minimum.value = v; minimum.x = x; minimum.y = y; }
            }
        }
    }

    // Diverging colormap like Fig. 2: blue below zero, white at zero, green above
    function color(v) {
        var t, c;
        if (v < 0) { t = Math.min(-v / 25, 1); c = [255 - t * (255 - 33), 255 - t * (255 - 102), 255 - t * (255 - 172)]; }
        else { t = Math.min(v / 70, 1); c = [255 - t * (255 - 150), 255 - t * (255 - 200), 255 - t * (255 - 110)]; }
        return c;
    }

    var heat = document.createElement('canvas');
    heat.width = GW;
    heat.height = GH;
    var heatCtx = heat.getContext('2d');
    var heatImage = heatCtx.createImageData(GW, GH);

    function drawContour(level, style, width) {
        // Marching squares over the sampled objective
        ctx.beginPath();
        var sx = canvas.width / GW, sy = canvas.height / GH;
        for (var j = 0; j + 1 < GH; j++) {
            for (var i = 0; i + 1 < GW; i++) {
                var a = field[j * GW + i] - level, b = field[j * GW + i + 1] - level;
                var c = field[(j + 1) * GW + i + 1] - level, d = field[(j + 1) * GW + i] - level;
                var pts = [];
                if ((a < 0) !== (b < 0)) pts.push([i + a / (a - b), j]);
                if ((b < 0) !== (c < 0)) pts.push([i + 1, j + b / (b - c)]);
                if ((d < 0) !== (c < 0)) pts.push([i + d / (d - c), j + 1]);
                if ((a < 0) !== (d < 0)) pts.push([i, j + a / (a - d)]);
                for (var k = 0; k + 1 < pts.length; k += 2) {
                    ctx.moveTo((pts[k][0] + 0.5) * sx, (pts[k][1] + 0.5) * sy);
                    ctx.lineTo((pts[k + 1][0] + 0.5) * sx, (pts[k + 1][1] + 0.5) * sy);
                }
            }
        }
        ctx.strokeStyle = style;
        ctx.lineWidth = width;
        ctx.stroke();
    }

    function handlePosition(b) {
        return [b.cx + Math.cos(b.angle) * (b.hx + 10), b.cy + Math.sin(b.angle) * (b.hx + 10)];
    }

    function draw() {
        var dpr = canvas.width / canvas.getBoundingClientRect().width;
        for (var k = 0; k < GW * GH; k++) {
            var c = color(field[k]);
            heatImage.data[4 * k] = c[0];
            heatImage.data[4 * k + 1] = c[1];
            heatImage.data[4 * k + 2] = c[2];
            heatImage.data[4 * k + 3] = 255;
        }
        heatCtx.putImageData(heatImage, 0, 0);
        ctx.imageSmoothingEnabled = true;
        ctx.drawImage(heat, 0, 0, canvas.width, canvas.height);

        for (var level = -40; level <= 80; level += 10)
            if (level !== 0)
                drawContour(level, 'rgba(20, 30, 40, 0.16)', dpr);
        drawContour(0, '#111', 2 * dpr);

        // Boxes and their rotation handles
        state.boxes.forEach(function (b) {
            var c = Math.cos(b.angle), s = Math.sin(b.angle);
            ctx.beginPath();
            [[-1, -1], [1, -1], [1, 1], [-1, 1]].forEach(function (q, i) {
                var p = toCanvas(b.cx + c * q[0] * b.hx - s * q[1] * b.hy, b.cy + s * q[0] * b.hx + c * q[1] * b.hy);
                if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
            });
            ctx.closePath();
            ctx.setLineDash(b.dashed ? [8 * dpr, 6 * dpr] : []);
            ctx.strokeStyle = b.color;
            ctx.lineWidth = 2.2 * dpr;
            ctx.stroke();
            ctx.setLineDash([]);
            var h = toCanvas.apply(null, handlePosition(b));
            ctx.beginPath();
            ctx.arc(h[0], h[1], 6 * dpr, 0, 2 * Math.PI);
            ctx.fillStyle = '#fff';
            ctx.fill();
            ctx.stroke();
        });

        // Cached points
        state.points.forEach(function (p) {
            var q = toCanvas(p.x, p.y);
            ctx.beginPath();
            ctx.arc(q[0], q[1], 6 * dpr, 0, 2 * Math.PI);
            ctx.strokeStyle = '#e8871e';
            ctx.lineWidth = 2.5 * dpr;
            ctx.stroke();
        });

        // Global minimum of the sampled objective
        var m = toCanvas(minimum.x, minimum.y), r = 7 * dpr;
        ctx.beginPath();
        ctx.moveTo(m[0] - r, m[1] - r); ctx.lineTo(m[0] + r, m[1] + r);
        ctx.moveTo(m[0] + r, m[1] - r); ctx.lineTo(m[0] - r, m[1] + r);
        ctx.strokeStyle = '#111';
        ctx.lineWidth = 2.5 * dpr;
        ctx.stroke();

        if (trace)
            drawTrace(dpr);
    }

    function drawTrace(dpr) {
        var frames = trace.frames, last = Math.min(traceFrame, frames.length - 1);
        ctx.lineWidth = 1.5 * dpr;
        if (trace.kind === 'nm') {
            for (var i = Math.max(0, last - 6); i <= last; i++) {
                var alpha = i === last ? 0.9 : 0.25;
                ctx.beginPath();
                frames[i].simplex.forEach(function (p, k) {
                    var q = toCanvas(p[0], p[1]);
                    if (k === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]);
                });
                ctx.closePath();
                ctx.fillStyle = 'rgba(232, 135, 30, ' + (alpha * 0.35) + ')';
                ctx.strokeStyle = 'rgba(160, 80, 10, ' + alpha + ')';
                ctx.fill();
                ctx.stroke();
            }
        }
        if (trace.kind === 'ellipsoid') {
            for (var i = Math.max(0, last - 4); i <= last; i++) {
                var f = frames[i], E = f.E;
                // Principal axes of E for drawing the ellipse {y : (y - c)^T E^-1 (y - c) <= 1}
                var tr = E[0] + E[3], det = E[0] * E[3] - E[1] * E[2];
                var l1 = tr / 2 + Math.sqrt(Math.max(tr * tr / 4 - det, 0)), l2 = tr / 2 - Math.sqrt(Math.max(tr * tr / 4 - det, 0));
                var angle = Math.abs(E[1]) > 1e-12 ? Math.atan2(l1 - E[0], E[1]) : (E[0] >= E[3] ? 0 : Math.PI / 2);
                var c = toCanvas(f.center[0], f.center[1]);
                var sx = canvas.width / (X1 - X0);
                ctx.beginPath();
                ctx.ellipse(c[0], c[1], Math.sqrt(Math.max(l1, 0)) * sx, Math.sqrt(Math.max(l2, 0)) * sx, -angle, 0, 2 * Math.PI);
                ctx.strokeStyle = 'rgba(160, 80, 10, ' + (i === last ? 0.9 : 0.25) + ')';
                ctx.stroke();
            }
        }
        if (trace.kind !== 'nm') {
            ctx.beginPath();
            for (var i = 0; i <= last; i++) {
                var p = frames[i].center || frames[i].point, q = toCanvas(p[0], p[1]);
                if (i === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]);
            }
            ctx.strokeStyle = 'rgba(160, 80, 10, 0.9)';
            ctx.stroke();
        }
        if (traceFrame >= frames.length - 1) {
            var x = toCanvas(trace.x[0], trace.x[1]);
            ctx.beginPath();
            ctx.arc(x[0], x[1], 5 * dpr, 0, 2 * Math.PI);
            ctx.fillStyle = '#e8871e';
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.stroke();
        }
    }

    function update() {
        sampleField();
        draw();
    }

    var readout = document.getElementById('readout');
    var names = { nm: 'Nelder-Mead', ellipsoid: 'Ellipsoid', sgd: 'SGD' };

    function run(x, y) {
        clearInterval(traceTimer);
        var optimize = { nm: nelderMead, ellipsoid: ellipsoidMethod, sgd: gradientDescent }[settings.optimizer];
        trace = optimize(x, y);
        trace.kind = settings.optimizer;
        trace.seed = [x, y];
        traceFrame = 0;
        var value = objective(trace.x[0], trace.x[1])[0];
        readout.textContent = names[trace.kind] + ': ' + trace.iterations + ' iterations, ' + trace.evaluations +
            ' evaluations\ng(x*) = ' + value.toFixed(2) + ' at (' + trace.x[0].toFixed(1) + ', ' + trace.x[1].toFixed(1) + ')' +
            '\nglobal min = ' + minimum.value.toFixed(2);
        readout.style.whiteSpace = 'pre-line';
        traceTimer = setInterval(function () {
            traceFrame++;
            draw();
            if (traceFrame >= trace.frames.length - 1)
                clearInterval(traceTimer);
        }, 90);
        draw();
    }

    function clearTrace() {
        clearInterval(traceTimer);
        trace = null;
    }

    // Pointer interaction

    function hitTest(x, y) {
        var tolerance = pixelsToWorld(10);
        for (var i = 0; i < state.points.length; i++)
            if (Math.hypot(x - state.points[i].x, y - state.points[i].y) < tolerance)
                return { type: 'point', index: i };
        for (var i = 0; i < 2; i++) {
            var h = handlePosition(state.boxes[i]);
            if (Math.hypot(x - h[0], y - h[1]) < tolerance)
                return { type: 'rotate', index: i };
        }
        for (var i = 1; i >= 0; i--)
            if (sdBox(state.boxes[i], x, y)[0] < 0)
                return { type: 'move', index: i };
        return null;
    }

    canvas.addEventListener('pointerdown', function (e) {
        var w = toWorld(e.clientX, e.clientY);
        var hit = hitTest(w[0], w[1]);
        canvas.setPointerCapture(e.pointerId);
        if (hit && hit.type === 'point') {
            state.points.splice(hit.index, 1);
            clearTrace();
            update();
            return;
        }
        if (hit) {
            var b = state.boxes[hit.index];
            drag = { type: hit.type, index: hit.index, dx: w[0] - b.cx, dy: w[1] - b.cy, start: w, moved: false };
            return;
        }
        drag = { type: 'click', start: w, moved: false };
    });

    canvas.addEventListener('pointermove', function (e) {
        var w = toWorld(e.clientX, e.clientY);
        if (!drag) {
            var hit = hitTest(w[0], w[1]);
            canvas.style.cursor = hit ? (hit.type === 'move' ? 'grab' : 'pointer') : 'crosshair';
            return;
        }
        if (Math.hypot(w[0] - drag.start[0], w[1] - drag.start[1]) > pixelsToWorld(3))
            drag.moved = true;
        if (!drag.moved || drag.type === 'click')
            return;
        var b = state.boxes[drag.index];
        if (drag.type === 'move') {
            b.cx = w[0] - drag.dx;
            b.cy = w[1] - drag.dy;
        } else {
            b.angle = Math.atan2(w[1] - b.cy, w[0] - b.cx);
        }
        clearTrace();
        update();
    });

    canvas.addEventListener('pointerup', function () {
        // A click without dragging runs the optimizer or adds a cached point, also on top of a box
        if (drag && !drag.moved && drag.type !== 'rotate') {
            var w = drag.start;
            if (settings.click === 'point') {
                state.points.push({ x: w[0], y: w[1] });
                clearTrace();
                update();
            } else {
                run(w[0], w[1]);
            }
        }
        drag = null;
    });

    // Controls

    function segmented(id, onChange) {
        var container = document.getElementById(id);
        container.querySelectorAll('button').forEach(function (button) {
            button.addEventListener('click', function () {
                container.querySelectorAll('button').forEach(function (b) { b.classList.toggle('active', b === button); });
                onChange(button.dataset.value);
            });
        });
    }

    segmented('objective-mode', function (value) {
        settings.softmax = value === 'softmax';
        document.getElementById('epsilon').disabled = !settings.softmax;
        clearTrace();
        update();
    });
    segmented('optimizer', function (value) {
        settings.optimizer = value;
        if (trace)
            run(trace.seed[0], trace.seed[1]);
    });
    segmented('click-mode', function (value) {
        settings.click = value;
    });

    var epsilon = document.getElementById('epsilon'), alpha = document.getElementById('alpha');
    epsilon.addEventListener('input', function () {
        settings.epsilon = parseFloat(epsilon.value);
        document.getElementById('epsilon-value').textContent = epsilon.value;
        clearTrace();
        update();
    });
    alpha.addEventListener('input', function () {
        settings.alpha = parseFloat(alpha.value);
        document.getElementById('alpha-value').textContent = settings.alpha.toFixed(2);
        clearTrace();
        update();
    });

    document.getElementById('clear-points').addEventListener('click', function () {
        state.points = [];
        clearTrace();
        update();
    });
    document.getElementById('reset-shapes').addEventListener('click', function () {
        state = defaults();
        clearTrace();
        readout.textContent = 'Click the plot to minimize g from that point.';
        update();
    });

    // Keep the canvas resolution matched to its displayed size
    function resize() {
        var rect = canvas.getBoundingClientRect();
        var dpr = Math.min(window.devicePixelRatio || 1, 2);
        canvas.width = Math.max(1, Math.round(rect.width * dpr));
        canvas.height = Math.max(1, Math.round(rect.height * dpr));
        draw();
    }
    new ResizeObserver(resize).observe(canvas);
    sampleField();
    resize();
})();
