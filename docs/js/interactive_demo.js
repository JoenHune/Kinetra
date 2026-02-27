/**
 * Kinetra Interactive Demo — WebAssembly Edition
 *
 * All planning and trajectory optimisation runs in the ACTUAL C++ library
 * compiled to WebAssembly via Emscripten.  Zero algorithms are reimplemented
 * in JavaScript — this file only handles:
 *   1. Canvas rendering (obstacles, paths, trajectories, heading arrows)
 *   2. UI interaction (click placement, sliders, state inputs)
 *   3. Calling the C++ Wasm module and parsing JSON results
 *   4. Plotly velocity profile rendering
 */

/* global KinetraModule, Plotly */

// ═════════════════════════════════════════════════════════════════════════
// State
// ═════════════════════════════════════════════════════════════════════════
const demo = {
    canvas: null,
    ctx: null,
    dpr: 1,
    wasm: null,
    wasmReady: false,

    world: { xmin: -10, xmax: 10, ymin: -7, ymax: 7 },

    mode: 'start',
    start: null,
    goal: null,
    obstacles: [],
    obsRadius: 0.8,

    rrtIterations: 3000,
    ilqrHorizon: 50,
    model: 'DiffDriveSimple',
    optimizer: 'iLQR',

    // MPCC weights
    mpccHorizon: 30,
    wContour: 50, wLag: 20, wHeading: 30, wProgress: 5, wControl: 1,
    mpccData: null,   // MPCC-specific result data

    rrtPath: null,
    optPath: null,
    rrtResult: null,
    optResult: null,
    running: false,
};

// ═════════════════════════════════════════════════════════════════════════
// Coordinate transforms
// ═════════════════════════════════════════════════════════════════════════
function worldToCanvas(wx, wy) {
    const cw = demo.canvas.width / demo.dpr;
    const ch = demo.canvas.height / demo.dpr;
    const { xmin, xmax, ymin, ymax } = demo.world;
    return [((wx - xmin) / (xmax - xmin)) * cw,
            ((ymax - wy) / (ymax - ymin)) * ch];
}

function canvasToWorld(cx, cy) {
    const cw = demo.canvas.width / demo.dpr;
    const ch = demo.canvas.height / demo.dpr;
    const { xmin, xmax, ymin, ymax } = demo.world;
    return [xmin + (cx / cw) * (xmax - xmin),
            ymax - (cy / ch) * (ymax - ymin)];
}

function worldScale(dist) {
    const cw = demo.canvas.width / demo.dpr;
    return (dist / (demo.world.xmax - demo.world.xmin)) * cw;
}

// ═════════════════════════════════════════════════════════════════════════
// Theme-aware colours
// ═════════════════════════════════════════════════════════════════════════
function getColor(name) {
    return getComputedStyle(document.documentElement).getPropertyValue('--' + name).trim();
}

// ═════════════════════════════════════════════════════════════════════════
// Drawing
// ═════════════════════════════════════════════════════════════════════════
function drawAll() {
    const ctx = demo.ctx;
    const cw = demo.canvas.width / demo.dpr;
    const ch = demo.canvas.height / demo.dpr;

    ctx.save();
    ctx.scale(demo.dpr, demo.dpr);

    ctx.fillStyle = getColor('canvas-bg');
    ctx.fillRect(0, 0, cw, ch);

    drawGrid(ctx, cw, ch);
    drawObstacles(ctx);
    drawPath(ctx, demo.rrtPath, getColor('text-muted'), 1.5, true);
    drawPath(ctx, demo.optPath, getColor('accent'), 2.5, false);
    drawHeadingArrows(ctx, demo.optPath || demo.rrtPath);
    drawEndpoint(ctx, demo.start, getColor('green'), 'S');
    drawEndpoint(ctx, demo.goal, getColor('red'), 'G');

    ctx.restore();
}

function drawGrid(ctx, cw, ch) {
    ctx.strokeStyle = getColor('grid-line');
    ctx.lineWidth = 0.5;
    ctx.setLineDash([2, 4]);
    const { xmin, xmax, ymin, ymax } = demo.world;
    for (let x = Math.ceil(xmin); x <= xmax; x++) {
        const [px] = worldToCanvas(x, 0);
        ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, ch); ctx.stroke();
    }
    for (let y = Math.ceil(ymin); y <= ymax; y++) {
        const [, py] = worldToCanvas(0, y);
        ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(cw, py); ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.strokeStyle = getColor('border');
    ctx.lineWidth = 1;
    const [ox, oy] = worldToCanvas(0, 0);
    ctx.beginPath(); ctx.moveTo(ox, 0); ctx.lineTo(ox, ch); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, oy); ctx.lineTo(cw, oy); ctx.stroke();
}

function drawObstacles(ctx) {
    ctx.fillStyle = getColor('red') + '40';
    ctx.strokeStyle = getColor('red') + '90';
    ctx.lineWidth = 1.5;
    for (const obs of demo.obstacles) {
        const [cx, cy] = worldToCanvas(obs.cx, obs.cy);
        const r = worldScale(obs.r);
        ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
}

function drawPath(ctx, path, color, width, dashed) {
    if (!path || path.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dashed ? [6, 4] : []);
    ctx.beginPath();
    const [sx, sy] = worldToCanvas(path[0].x, path[0].y);
    ctx.moveTo(sx, sy);
    for (let i = 1; i < path.length; i++) {
        const [px, py] = worldToCanvas(path[i].x, path[i].y);
        ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.setLineDash([]);
}

function drawHeadingArrows(ctx, path) {
    if (!path || path.length < 2) return;
    const step = Math.max(1, Math.floor(path.length / 20));
    ctx.strokeStyle = getColor('green');
    ctx.lineWidth = 1.5;
    const arrowLen = worldScale(0.4);
    for (let i = 0; i < path.length; i += step) {
        const [px, py] = worldToCanvas(path[i].x, path[i].y);
        const th = path[i].theta || 0;
        const ex = px + arrowLen * Math.cos(-th);
        const ey = py + arrowLen * Math.sin(-th);
        ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(ex, ey); ctx.stroke();
        const hl = arrowLen * 0.3;
        ctx.beginPath();
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - hl * Math.cos(-th - 0.4), ey - hl * Math.sin(-th - 0.4));
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - hl * Math.cos(-th + 0.4), ey - hl * Math.sin(-th + 0.4));
        ctx.stroke();
    }
}

function drawEndpoint(ctx, point, color, label) {
    if (!point) return;
    const [px, py] = worldToCanvas(point.x, point.y);
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(px, py, 8, 0, Math.PI * 2); ctx.fill();
    if (point.theta !== undefined) {
        const len = worldScale(0.6);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(px + len * Math.cos(-point.theta), py + len * Math.sin(-point.theta));
        ctx.stroke();
    }
    ctx.fillStyle = getColor('canvas-bg');
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, px, py);
}

// ═════════════════════════════════════════════════════════════════════════
// Wasm initialisation
// ═════════════════════════════════════════════════════════════════════════
async function initWasm() {
    const statusEl = document.getElementById('wasm-status');
    try {
        demo.wasm = await KinetraModule();
        demo.wasmReady = true;
        statusEl.textContent = 'WASM Ready';
        statusEl.className = 'wasm-status ready';
        enablePlanButtons(true);
        console.log('Kinetra WASM loaded:', JSON.parse(demo.wasm.getVersion()));
    } catch (e) {
        statusEl.textContent = 'WASM Error';
        statusEl.className = 'wasm-status error';
        console.error('Failed to load Kinetra WASM:', e);
    }
}

function enablePlanButtons(en) {
    document.getElementById('btn-plan').disabled = !en;
    document.getElementById('btn-rrt-only').disabled = !en;
    document.getElementById('btn-opt-only').disabled = !en;
}

// ═════════════════════════════════════════════════════════════════════════
// Build JSON problem for C++ Wasm
// ═════════════════════════════════════════════════════════════════════════
function buildProblemJSON(extra) {
    const s = demo.start || { x: -7, y: 0, theta: 0 };
    const g = demo.goal  || { x: 7, y: 0, theta: 0 };
    return {
        start: { x: s.x, y: s.y, theta: s.theta || 0 },
        goal:  { x: g.x, y: g.y, theta: g.theta || 0 },
        environment: {
            bounds: { min: [demo.world.xmin, demo.world.ymin], max: [demo.world.xmax, demo.world.ymax] },
            obstacles: demo.obstacles.map(o => ({ type: 'circle', center: [o.cx, o.cy], radius: o.r })),
        },
        options: { maxIterations: demo.rrtIterations, timeLimitMs: 10000, goalTolerance: 0.5 },
        ...extra,
    };
}

// ═════════════════════════════════════════════════════════════════════════
// Planning entry points (calls C++ via Wasm)
// ═════════════════════════════════════════════════════════════════════════
async function runPipeline() {
    if (!demo.wasmReady || demo.running) return;
    demo.running = true;
    enablePlanButtons(false);
    setStatus('Running RRT* + ' + demo.optimizer + '...');
    setPipelineState('rrt');
    demo.mpccData = null;

    if (demo.optimizer === 'MPCC') {
        // Use MPCC-specific pipeline
        const json = buildProblemJSON({
            model: demo.model,
            rrtOptions: { maxIterations: demo.rrtIterations, stepSize: 0.5, goalBias: 0.1 },
            mpccOptions: {
                horizon: demo.mpccHorizon, dt: 0.05,
                wContour: demo.wContour, wLag: demo.wLag,
                wProgress: demo.wProgress,
            },
        });
        try {
            const result = JSON.parse(demo.wasm.planPipelineMPCC(JSON.stringify(json)));
            if (result.error) { setStatus('Error: ' + result.error); }
            else {
                setPipelineState('opt');
                demo.rrtResult = result.rrt;
                demo.rrtPath = result.rrt?.trajectory || null;
                demo.optResult = result.optimized;
                demo.optPath = result.optimized?.trajectory || null;
                demo.mpccData = result.mpcc || null;
                updateRRTStats(result.rrt);
                updateOptStats(result.optimized);
                renderVelocityProfile(demo.optPath || demo.rrtPath);
                renderMPCCAnalysis(demo.mpccData);
                setPipelineState('done');
                setStatus('Pipeline complete — RRT*: ' +
                    (result.rrt?.solve_time_ms?.toFixed(1) || '?') + 'ms, MPCC: ' +
                    (result.optimized?.solve_time_ms?.toFixed(1) || '?') + 'ms' +
                    (result.mpcc?.sqpIterations ? (' (' + result.mpcc.sqpIterations + ' SQP iters)') : ''));
            }
        } catch (e) { setStatus('Error: ' + e.message); }
    } else {
        // Original iLQR/STOMP pipeline
        const json = buildProblemJSON({
            model: demo.model, optimizer: demo.optimizer,
            rrtOptions: { maxIterations: demo.rrtIterations, stepSize: 0.5, goalBias: 0.1 },
            ilqrOptions: { horizon: demo.ilqrHorizon, dt: 0.05 },
            stompOptions: { numRollouts: 30, noiseSigma: 0.5 },
        });
        try {
            const result = JSON.parse(demo.wasm.planPipeline(JSON.stringify(json)));
            if (result.error) { setStatus('Error: ' + result.error); }
            else {
                setPipelineState('opt');
                demo.rrtResult = result.rrt;
                demo.rrtPath = result.rrt?.trajectory || null;
                demo.optResult = result.optimized;
                demo.optPath = result.optimized?.trajectory || null;
                updateRRTStats(result.rrt);
                updateOptStats(result.optimized);
                renderVelocityProfile(demo.optPath || demo.rrtPath);
                hideMPCCAnalysis();
                setPipelineState('done');
                setStatus('Pipeline complete — RRT*: ' +
                    (result.rrt?.solve_time_ms?.toFixed(1) || '?') + 'ms, ' +
                    demo.optimizer + ': ' +
                    (result.optimized?.solve_time_ms?.toFixed(1) || '?') + 'ms');
            }
        } catch (e) { setStatus('Error: ' + e.message); }
    }

    demo.running = false;
    enablePlanButtons(true);
    drawAll();
}

async function runRRTOnly() {
    if (!demo.wasmReady || demo.running) return;
    demo.running = true;
    enablePlanButtons(false);
    setStatus('Running RRT*...');
    setPipelineState('rrt');

    const json = buildProblemJSON({
        plannerOptions: { maxIterations: demo.rrtIterations, stepSize: 0.5, goalBias: 0.1 },
    });

    try {
        const result = JSON.parse(demo.wasm.planRRTStar(JSON.stringify(json)));
        if (result.error) { setStatus('Error: ' + result.error); }
        else {
            const r = result.result || result;
            demo.rrtResult = r;
            demo.rrtPath = r.trajectory || null;
            demo.optPath = null; demo.optResult = null;
            updateRRTStats(r);
            clearOptStats();
            renderVelocityProfile(demo.rrtPath);
            setPipelineState('done');
            setStatus('RRT* complete — ' + (r.solve_time_ms?.toFixed(1) || '?') + 'ms');
        }
    } catch (e) { setStatus('Error: ' + e.message); }

    demo.running = false;
    enablePlanButtons(true);
    drawAll();
}

async function runOptOnly() {
    if (!demo.wasmReady || demo.running) return;
    demo.running = true;
    enablePlanButtons(false);
    setStatus('Running ' + demo.optimizer + '...');
    setPipelineState('opt');
    demo.mpccData = null;

    const opt = demo.optimizer;

    try {
        let raw, result;
        if (opt === 'MPCC') {
            const json = buildProblemJSON({
                model: demo.model,
                plannerOptions: {
                    horizon: demo.mpccHorizon, dt: 0.05,
                    wContour: demo.wContour, wLag: demo.wLag,
                    wHeading: demo.wHeading, wProgress: demo.wProgress,
                    wControl: demo.wControl, sqpMaxIterations: 20,
                },
            });
            raw = demo.wasm.optimizeMPCC(JSON.stringify(json));
            result = JSON.parse(raw);
            if (result.error) { setStatus('Error: ' + result.error); }
            else {
                const r = result.result || result;
                demo.optResult = r;
                demo.optPath = r.trajectory || null;
                demo.mpccData = result.mpcc || null;
                updateOptStats(r);
                renderVelocityProfile(demo.optPath);
                renderMPCCAnalysis(demo.mpccData);
                setPipelineState('done');
                setStatus('MPCC complete — ' + (r.solve_time_ms?.toFixed(1) || '?') + 'ms' +
                    (result.mpcc?.sqpIterations ? (' (' + result.mpcc.sqpIterations + ' SQP iters)') : ''));
            }
        } else {
            const json = opt === 'STOMP'
                ? buildProblemJSON({ plannerOptions: { numTimesteps: 60, numRollouts: 30, maxIterations: 100, noiseSigma: 0.5 } })
                : buildProblemJSON({ model: demo.model, plannerOptions: { horizon: demo.ilqrHorizon, dt: 0.05, maxIterations: 100 } });
            raw = opt === 'STOMP'
                ? demo.wasm.optimizeSTOMP(JSON.stringify(json))
                : demo.wasm.optimizeiLQR(JSON.stringify(json));
            result = JSON.parse(raw);
            if (result.error) { setStatus('Error: ' + result.error); }
            else {
                const r = result.result || result;
                demo.optResult = r;
                demo.optPath = r.trajectory || null;
                updateOptStats(r);
                renderVelocityProfile(demo.optPath);
                hideMPCCAnalysis();
                setPipelineState('done');
                setStatus(opt + ' complete — ' + (r.solve_time_ms?.toFixed(1) || '?') + 'ms');
            }
        }
    } catch (e) { setStatus('Error: ' + e.message); }

    demo.running = false;
    enablePlanButtons(true);
    drawAll();
}

// ═════════════════════════════════════════════════════════════════════════
// UI helpers
// ═════════════════════════════════════════════════════════════════════════
function setStatus(msg) { document.getElementById('demo-status').textContent = msg; }

function setPipelineState(state) {
    ['rrt', 'opt', 'done'].forEach((s, i) => {
        const el = document.getElementById('pipe-' + s);
        el.classList.remove('active', 'done');
    });
    const idx = ['rrt', 'opt', 'done'].indexOf(state);
    for (let i = 0; i < idx; i++) document.getElementById('pipe-' + ['rrt', 'opt', 'done'][i]).classList.add('done');
    if (idx >= 0) document.getElementById('pipe-' + ['rrt', 'opt', 'done'][idx]).classList.add('active');
}

function updateRRTStats(r) {
    if (!r) return;
    document.getElementById('rrt-stat-nodes').textContent = r.iterations || '-';
    document.getElementById('rrt-stat-length').textContent = r.path_length != null ? r.path_length.toFixed(2) : '-';
    document.getElementById('rrt-stat-time').textContent = r.solve_time_ms != null ? r.solve_time_ms.toFixed(1) : '-';
}

function updateOptStats(r) {
    if (!r) { clearOptStats(); return; }
    document.getElementById('opt-stat-cost').textContent = r.cost != null ? r.cost.toFixed(2) : '-';
    document.getElementById('opt-stat-smooth').textContent = r.smoothness != null ? r.smoothness.toFixed(4) : '-';
    document.getElementById('opt-stat-time').textContent = r.solve_time_ms != null ? r.solve_time_ms.toFixed(1) : '-';
    document.getElementById('opt-stat-curv').textContent = r.max_curvature != null ? r.max_curvature.toFixed(3) : '-';
}

function clearOptStats() {
    ['opt-stat-cost', 'opt-stat-smooth', 'opt-stat-time', 'opt-stat-curv']
        .forEach(id => document.getElementById(id).textContent = '-');
}

// ═════════════════════════════════════════════════════════════════════════
// Plotly velocity profile
// ═════════════════════════════════════════════════════════════════════════
function getPlotlyTheme() {
    const dk = document.documentElement.getAttribute('data-theme') !== 'light';
    return {
        paper_bgcolor: dk ? '#0d1117' : '#ffffff',
        plot_bgcolor:  dk ? '#161b22' : '#f6f8fa',
        font:          { color: dk ? '#c9d1d9' : '#24292f' },
        gridcolor:     dk ? '#30363d' : '#d0d7de',
    };
}

function renderVelocityProfile(path) {
    const el = document.getElementById('plot-demo-velocity');
    if (!path || path.length < 2) { Plotly.purge(el); return; }
    const times = [], vels = [];
    for (let i = 1; i < path.length; i++) {
        const dx = path[i].x - path[i-1].x, dy = path[i].y - path[i-1].y;
        const t1 = path[i].t || i * 0.1, t0 = path[i-1].t || (i-1) * 0.1;
        const dt = t1 - t0;
        times.push(t1);
        vels.push(dt > 0 ? Math.sqrt(dx*dx + dy*dy) / dt : 0);
    }
    const th = getPlotlyTheme();
    Plotly.newPlot(el, [{
        x: times, y: vels, mode: 'lines',
        line: { color: getColor('accent'), width: 2 },
        fill: 'tozeroy', fillcolor: getColor('accent') + '20', name: 'Velocity',
    }], {
        paper_bgcolor: th.paper_bgcolor, plot_bgcolor: th.plot_bgcolor,
        font: th.font, margin: { l: 50, r: 20, t: 10, b: 40 },
        xaxis: { gridcolor: th.gridcolor, title: 'Time (s)' },
        yaxis: { gridcolor: th.gridcolor, title: 'Velocity (m/s)' },
    }, { responsive: true, displayModeBar: false });
}

window.updatePlotlyTheme = function() {
    renderVelocityProfile(demo.optPath || demo.rrtPath);
    if (demo.mpccData) renderMPCCAnalysis(demo.mpccData);
};

// ═════════════════════════════════════════════════════════════════════════
// MPCC Analysis Plots
// ═════════════════════════════════════════════════════════════════════════
function renderMPCCAnalysis(mpcc) {
    const card = document.getElementById('mpcc-analysis-card');
    if (!mpcc || !card) return;
    card.style.display = '';

    const th = getPlotlyTheme();
    const layout = (title, yLabel) => ({
        paper_bgcolor: th.paper_bgcolor, plot_bgcolor: th.plot_bgcolor,
        font: th.font, margin: { l: 50, r: 20, t: 30, b: 40 },
        xaxis: { gridcolor: th.gridcolor, title: 'Step k' },
        yaxis: { gridcolor: th.gridcolor, title: yLabel },
        title: { text: title, font: { size: 13 } },
        legend: { orientation: 'h', y: -0.25 },
    });

    // Contour + Lag + Heading errors
    const steps = mpcc.contourErrors ? mpcc.contourErrors.map((_, i) => i) : [];
    const errEl = document.getElementById('plot-mpcc-errors');
    Plotly.newPlot(errEl, [
        { x: steps, y: mpcc.contourErrors, name: 'Contour eₒ', line: { color: '#58a6ff', width: 2 } },
        { x: steps, y: mpcc.lagErrors, name: 'Lag eₗ', line: { color: '#f0883e', width: 2 } },
        { x: steps, y: mpcc.headingErrors, name: 'Heading eθ', line: { color: '#a371f7', width: 2 } },
    ], layout('Tracking Errors', 'Error'), { responsive: true, displayModeBar: false });

    // Progress along path
    const progEl = document.getElementById('plot-mpcc-progress');
    const progSteps = mpcc.progress ? mpcc.progress.map((_, i) => i) : [];
    Plotly.newPlot(progEl, [
        { x: progSteps, y: mpcc.progress, name: 'Progress s(k)', line: { color: '#3fb950', width: 2 },
          fill: 'tozeroy', fillcolor: '#3fb95020' },
    ], layout('Path Progress', 's (arc-length)'), { responsive: true, displayModeBar: false });
}

function hideMPCCAnalysis() {
    const card = document.getElementById('mpcc-analysis-card');
    if (card) card.style.display = 'none';
}

// ═════════════════════════════════════════════════════════════════════════
// Optimizer select → show/hide MPCC weight panel
// ═════════════════════════════════════════════════════════════════════════
function toggleMPCCPanel() {
    const mpccPanel = document.getElementById('mpcc-weights');
    if (mpccPanel) mpccPanel.style.display = demo.optimizer === 'MPCC' ? '' : 'none';
}

// ═════════════════════════════════════════════════════════════════════════
// Presets
// ═════════════════════════════════════════════════════════════════════════
const PRESETS = {
    narrowPassage: {
        start: { x: -7, y: 0, theta: 0 }, goal: { x: 7, y: 0, theta: 0 },
        obstacles: [{ cx: 0, cy: 3, r: 2.5 }, { cx: 0, cy: -3, r: 2.5 }],
    },
    uTurn: {
        start: { x: -5, y: -4, theta: 0 }, goal: { x: -5, y: 4, theta: Math.PI },
        obstacles: [{ cx: -2, cy: 0, r: 3 }, { cx: 3, cy: 3, r: 1.5 }, { cx: 3, cy: -3, r: 1.5 }],
    },
    clutteredField: {
        start: { x: -8, y: -5, theta: 0.3 }, goal: { x: 8, y: 5, theta: -0.3 },
        obstacles: [
            { cx: -4, cy: 2, r: 1.2 }, { cx: -2, cy: -3, r: 1 },
            { cx: 0, cy: 1, r: 0.8 }, { cx: 2, cy: -1, r: 1.1 },
            { cx: 4, cy: 3, r: 1 }, { cx: 5, cy: -4, r: 1.3 },
            { cx: -6, cy: -2, r: 0.9 }, { cx: 1, cy: 4, r: 0.7 },
        ],
    },
    parallelParking: {
        start: { x: -7, y: 2, theta: 0 }, goal: { x: 0, y: -2, theta: Math.PI / 2 },
        obstacles: [{ cx: -2, cy: -2, r: 1.2 }, { cx: 2, cy: -2, r: 1.2 }, { cx: 0, cy: -5, r: 1.5 }],
    },
};

function applyPreset(name) {
    const p = PRESETS[name]; if (!p) return;
    demo.start = { ...p.start }; demo.goal = { ...p.goal };
    demo.obstacles = p.obstacles.map(o => ({ ...o }));
    demo.rrtPath = null; demo.optPath = null;
    demo.rrtResult = null; demo.optResult = null;
    syncStateInputs(); drawAll();
    setStatus('Preset loaded — click Plan & Optimize to run.');
}

function syncStateInputs() {
    const s = demo.start || { x: -7, y: 0, theta: 0 };
    const g = demo.goal || { x: 7, y: 0, theta: 0 };
    document.getElementById('start-x').value = s.x;
    document.getElementById('start-y').value = s.y;
    document.getElementById('start-theta').value = s.theta.toFixed(2);
    document.getElementById('goal-x').value = g.x;
    document.getElementById('goal-y').value = g.y;
    document.getElementById('goal-theta').value = g.theta.toFixed(2);
}

function readStateInputs() {
    demo.start = {
        x: parseFloat(document.getElementById('start-x').value) || -7,
        y: parseFloat(document.getElementById('start-y').value) || 0,
        theta: parseFloat(document.getElementById('start-theta').value) || 0,
    };
    demo.goal = {
        x: parseFloat(document.getElementById('goal-x').value) || 7,
        y: parseFloat(document.getElementById('goal-y').value) || 0,
        theta: parseFloat(document.getElementById('goal-theta').value) || 0,
    };
}

// ═════════════════════════════════════════════════════════════════════════
// Initialisation
// ═════════════════════════════════════════════════════════════════════════
function initDemo() {
    demo.canvas = document.getElementById('demo-canvas');
    demo.ctx = demo.canvas.getContext('2d');

    demo.dpr = window.devicePixelRatio || 1;
    const rect = demo.canvas.getBoundingClientRect();
    demo.canvas.width = rect.width * demo.dpr;
    demo.canvas.height = rect.height * demo.dpr;

    demo.start = { x: -7, y: 0, theta: 0 };
    demo.goal  = { x: 7, y: 0, theta: 0 };
    syncStateInputs();

    // Canvas click
    demo.canvas.addEventListener('click', (e) => {
        const r = demo.canvas.getBoundingClientRect();
        const [wx, wy] = canvasToWorld(e.clientX - r.left, e.clientY - r.top);
        if (demo.mode === 'start') {
            demo.start = { x: wx, y: wy, theta: demo.start?.theta || 0 };
            syncStateInputs();
            setStatus('Start placed at (' + wx.toFixed(1) + ', ' + wy.toFixed(1) + ')');
        } else if (demo.mode === 'goal') {
            demo.goal = { x: wx, y: wy, theta: demo.goal?.theta || 0 };
            syncStateInputs();
            setStatus('Goal placed at (' + wx.toFixed(1) + ', ' + wy.toFixed(1) + ')');
        } else if (demo.mode === 'obstacle') {
            demo.obstacles.push({ cx: wx, cy: wy, r: demo.obsRadius });
            setStatus('Obstacle added, r=' + demo.obsRadius);
        }
        drawAll();
    });

    // Right-click to remove obstacle
    demo.canvas.addEventListener('contextmenu', (e) => {
        e.preventDefault();
        const r = demo.canvas.getBoundingClientRect();
        const [wx, wy] = canvasToWorld(e.clientX - r.left, e.clientY - r.top);
        let minD = Infinity, minI = -1;
        demo.obstacles.forEach((o, i) => {
            const d = Math.hypot(o.cx - wx, o.cy - wy);
            if (d < minD) { minD = d; minI = i; }
        });
        if (minI >= 0 && minD < demo.obstacles[minI].r + 0.5) {
            demo.obstacles.splice(minI, 1);
            drawAll(); setStatus('Obstacle removed.');
        }
    });

    // Mode buttons
    document.querySelectorAll('#mode-group .btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('#mode-group .btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            demo.mode = btn.dataset.mode;
        });
    });

    // Sliders
    [['obs-radius', 'radius-val', v => { demo.obsRadius = parseFloat(v); return v; }],
     ['rrt-iters', 'rrt-iter-val', v => { demo.rrtIterations = parseInt(v); return v; }],
     ['ilqr-horizon', 'horizon-val', v => { demo.ilqrHorizon = parseInt(v); return v; }],
    ].forEach(([sid, vid, fn]) => {
        const s = document.getElementById(sid), v = document.getElementById(vid);
        s.addEventListener('input', () => { v.textContent = fn(s.value); });
    });

    // Selects
    document.getElementById('model-select').addEventListener('change', e => { demo.model = e.target.value; });
    document.getElementById('optimizer-select').addEventListener('change', e => {
        demo.optimizer = e.target.value;
        toggleMPCCPanel();
    });

    // MPCC weight sliders
    [['mpcc-horizon', 'mpcc-horizon-val', v => { demo.mpccHorizon = parseInt(v); return v; }],
     ['wContour', 'wc-val', v => { demo.wContour = parseFloat(v); return v; }],
     ['wLag', 'wl-val', v => { demo.wLag = parseFloat(v); return v; }],
     ['wHeading', 'wh-val', v => { demo.wHeading = parseFloat(v); return v; }],
     ['wProgress', 'wp-val', v => { demo.wProgress = parseFloat(v); return v; }],
     ['wControl', 'wu-val', v => { demo.wControl = parseFloat(v); return v; }],
    ].forEach(([sid, vid, fn]) => {
        const s = document.getElementById(sid), v = document.getElementById(vid);
        if (s && v) s.addEventListener('input', () => { v.textContent = fn(s.value); });
    });

    // State inputs
    ['start-x','start-y','start-theta','goal-x','goal-y','goal-theta'].forEach(id => {
        document.getElementById(id).addEventListener('change', () => { readStateInputs(); drawAll(); });
    });

    // Plan buttons
    document.getElementById('btn-plan').addEventListener('click', runPipeline);
    document.getElementById('btn-rrt-only').addEventListener('click', runRRTOnly);
    document.getElementById('btn-opt-only').addEventListener('click', runOptOnly);

    // Clear
    document.getElementById('btn-clear').addEventListener('click', () => {
        demo.obstacles = []; demo.rrtPath = null; demo.optPath = null;
        demo.rrtResult = null; demo.optResult = null; demo.mpccData = null;
        demo.start = { x: -7, y: 0, theta: 0 }; demo.goal = { x: 7, y: 0, theta: 0 };
        syncStateInputs(); clearOptStats();
        ['rrt-stat-nodes','rrt-stat-length','rrt-stat-time'].forEach(id => document.getElementById(id).textContent = '-');
        setPipelineState('');
        Plotly.purge(document.getElementById('plot-demo-velocity'));
        hideMPCCAnalysis();
        setStatus('Cleared.'); drawAll();
    });

    // Presets
    document.getElementById('btn-preset-1').addEventListener('click', () => applyPreset('narrowPassage'));
    document.getElementById('btn-preset-2').addEventListener('click', () => applyPreset('uTurn'));
    document.getElementById('btn-preset-3').addEventListener('click', () => applyPreset('clutteredField'));
    document.getElementById('btn-preset-4').addEventListener('click', () => applyPreset('parallelParking'));

    // Keyboard shortcuts
    document.addEventListener('keydown', (e) => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
        if (e.key === '1') { demo.mode = 'start'; updMode(); }
        if (e.key === '2') { demo.mode = 'goal'; updMode(); }
        if (e.key === '3') { demo.mode = 'obstacle'; updMode(); }
        if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); runPipeline(); }
        if (e.key === 'Escape' || e.key === 'c') { document.getElementById('btn-clear').click(); }
    });

    function updMode() {
        document.querySelectorAll('#mode-group .btn').forEach(b =>
            b.classList.toggle('active', b.dataset.mode === demo.mode));
    }

    window.addEventListener('resize', () => {
        const r = demo.canvas.getBoundingClientRect();
        demo.canvas.width = r.width * demo.dpr;
        demo.canvas.height = r.height * demo.dpr;
        drawAll();
    });

    drawAll();
    initWasm();
}

if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', initDemo);
else initDemo();
