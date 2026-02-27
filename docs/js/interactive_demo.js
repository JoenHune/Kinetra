/**
 * Kinetra Interactive Demo
 * ════════════════════════
 * A fully in-browser 2D motion planning playground.
 *
 * Features:
 *   • Click to place start / goal / circle obstacles
 *   • RRT* and Greedy Best-First Search algorithms
 *   • Animated tree growth & path rendering
 *   • Preset scenarios (Narrow Gap, Maze)
 *   • Adjustable parameters (step size, goal bias, speed, …)
 */

// ── State ────────────────────────────────────────────────────────────────────
const demo = {
    canvas: null,
    ctx: null,
    W: 900,   // logical canvas width (will scale to CSS)
    H: 600,
    // World bounds in metres
    world: { xmin: -10, xmax: 10, ymin: -7, ymax: 7 },
    mode: 'start',   // 'start' | 'goal' | 'obstacle'
    start: null,      // {x, y}
    goal: null,       // {x, y}
    obstacles: [],    // [{cx, cy, r}]
    tree: [],         // [{x, y, parent}]  (parent = index or -1)
    path: [],         // [{x, y}]
    planning: false,
    animHandle: null,
};

// ── Coordinate transforms ────────────────────────────────────────────────────
function worldToCanvas(wx, wy) {
    const { xmin, xmax, ymin, ymax } = demo.world;
    const px = ((wx - xmin) / (xmax - xmin)) * demo.W;
    const py = ((ymax - wy) / (ymax - ymin)) * demo.H;  // flip Y
    return [px, py];
}

function canvasToWorld(px, py) {
    const { xmin, xmax, ymin, ymax } = demo.world;
    const wx = xmin + (px / demo.W) * (xmax - xmin);
    const wy = ymax - (py / demo.H) * (ymax - ymin);
    return [wx, wy];
}

function worldDist(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
}

function worldScale(metres) {
    const { xmin, xmax } = demo.world;
    return (metres / (xmax - xmin)) * demo.W;
}

// ── Collision ────────────────────────────────────────────────────────────────
function pointFree(p) {
    for (const o of demo.obstacles) {
        if (Math.hypot(p.x - o.cx, p.y - o.cy) <= o.r) return false;
    }
    return true;
}

function segmentFree(a, b, steps = 20) {
    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const p = { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) };
        if (!pointFree(p)) return false;
    }
    return true;
}

// ── Drawing ──────────────────────────────────────────────────────────────────
function draw() {
    const ctx = demo.ctx;
    ctx.clearRect(0, 0, demo.W, demo.H);

    // Background grid
    ctx.strokeStyle = '#21262d';
    ctx.lineWidth = 0.5;
    const { xmin, xmax, ymin, ymax } = demo.world;
    for (let gx = Math.ceil(xmin); gx <= Math.floor(xmax); gx++) {
        const [px] = worldToCanvas(gx, 0);
        ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, demo.H); ctx.stroke();
    }
    for (let gy = Math.ceil(ymin); gy <= Math.floor(ymax); gy++) {
        const [, py] = worldToCanvas(0, gy);
        ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(demo.W, py); ctx.stroke();
    }

    // Obstacles
    for (const o of demo.obstacles) {
        const [cx, cy] = worldToCanvas(o.cx, o.cy);
        const r = worldScale(o.r);
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(248,81,73,0.25)';
        ctx.fill();
        ctx.strokeStyle = '#f85149';
        ctx.lineWidth = 1.5;
        ctx.stroke();
    }

    // Tree edges
    if (demo.tree.length > 1) {
        ctx.strokeStyle = 'rgba(48,54,61,0.7)';
        ctx.lineWidth = 0.8;
        for (let i = 1; i < demo.tree.length; i++) {
            const node = demo.tree[i];
            if (node.parent < 0) continue;
            const parent = demo.tree[node.parent];
            const [x1, y1] = worldToCanvas(parent.x, parent.y);
            const [x2, y2] = worldToCanvas(node.x, node.y);
            ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
        }
    }

    // Path
    if (demo.path.length >= 2) {
        ctx.strokeStyle = '#58a6ff';
        ctx.lineWidth = 3;
        ctx.shadowColor = '#58a6ff';
        ctx.shadowBlur = 8;
        ctx.beginPath();
        const [sx, sy] = worldToCanvas(demo.path[0].x, demo.path[0].y);
        ctx.moveTo(sx, sy);
        for (let i = 1; i < demo.path.length; i++) {
            const [px, py] = worldToCanvas(demo.path[i].x, demo.path[i].y);
            ctx.lineTo(px, py);
        }
        ctx.stroke();
        ctx.shadowBlur = 0;
    }

    // Start
    if (demo.start) {
        const [sx, sy] = worldToCanvas(demo.start.x, demo.start.y);
        ctx.beginPath();
        ctx.arc(sx, sy, 8, 0, Math.PI * 2);
        ctx.fillStyle = '#3fb950';
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    // Goal
    if (demo.goal) {
        const [gx, gy] = worldToCanvas(demo.goal.x, demo.goal.y);
        drawStar(ctx, gx, gy, 5, 10, 5);
        ctx.fillStyle = '#f85149';
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();
    }
}

function drawStar(ctx, cx, cy, spikes, outerR, innerR) {
    let rot = -Math.PI / 2;
    ctx.beginPath();
    for (let i = 0; i < spikes; i++) {
        ctx.lineTo(cx + Math.cos(rot) * outerR, cy + Math.sin(rot) * outerR);
        rot += Math.PI / spikes;
        ctx.lineTo(cx + Math.cos(rot) * innerR, cy + Math.sin(rot) * innerR);
        rot += Math.PI / spikes;
    }
    ctx.closePath();
}

// ── RRT* Algorithm ───────────────────────────────────────────────────────────
function rrtStarStep(tree, goal, opts) {
    const { xmin, xmax, ymin, ymax } = demo.world;
    const { stepSize, goalBias } = opts;

    // Sample random or goal-biased
    let sample;
    if (Math.random() < goalBias) {
        sample = { x: goal.x, y: goal.y };
    } else {
        sample = {
            x: xmin + Math.random() * (xmax - xmin),
            y: ymin + Math.random() * (ymax - ymin),
        };
    }

    // Find nearest node
    let nearestIdx = 0;
    let nearestDist = Infinity;
    for (let i = 0; i < tree.length; i++) {
        const d = worldDist(tree[i], sample);
        if (d < nearestDist) { nearestDist = d; nearestIdx = i; }
    }
    const nearest = tree[nearestIdx];

    // Steer
    const d = nearestDist;
    const newPt = d <= stepSize ? { ...sample } : {
        x: nearest.x + (sample.x - nearest.x) / d * stepSize,
        y: nearest.y + (sample.y - nearest.y) / d * stepSize,
    };

    if (!pointFree(newPt) || !segmentFree(nearest, newPt)) return null;

    // Near neighbors for rewire (RRT* radius)
    const gamma = 2.0 * Math.sqrt((xmax - xmin) * (ymax - ymin) / Math.PI);
    const rStar = Math.min(gamma * Math.pow(Math.log(tree.length + 1) / (tree.length + 1), 0.5), stepSize * 3);
    let bestParent = nearestIdx;
    let bestCost = (tree[nearestIdx].cost || 0) + worldDist(nearest, newPt);

    const nearIdxs = [];
    for (let i = 0; i < tree.length; i++) {
        if (worldDist(tree[i], newPt) < rStar) nearIdxs.push(i);
    }

    for (const ni of nearIdxs) {
        const nc = (tree[ni].cost || 0) + worldDist(tree[ni], newPt);
        if (nc < bestCost && segmentFree(tree[ni], newPt)) {
            bestCost = nc;
            bestParent = ni;
        }
    }

    const newNode = { x: newPt.x, y: newPt.y, parent: bestParent, cost: bestCost };
    const newIdx = tree.length;
    tree.push(newNode);

    // Rewire neighbors
    for (const ni of nearIdxs) {
        const rc = bestCost + worldDist(newPt, tree[ni]);
        if (rc < (tree[ni].cost || Infinity) && segmentFree(newPt, tree[ni])) {
            tree[ni].parent = newIdx;
            tree[ni].cost = rc;
        }
    }

    return newNode;
}

// ── Greedy Best-First (grid-based for fast demo) ─────────────────────────────
function greedyBestFirst(start, goal, obstacles) {
    const cellSize = 0.25;
    const { xmin, xmax, ymin, ymax } = demo.world;
    const cols = Math.ceil((xmax - xmin) / cellSize);
    const rows = Math.ceil((ymax - ymin) / cellSize);

    function toGrid(wx, wy) {
        return [Math.floor((wx - xmin) / cellSize), Math.floor((ymax - wy) / cellSize)];
    }
    function toWorld(gx, gy) {
        return { x: xmin + (gx + 0.5) * cellSize, y: ymax - (gy + 0.5) * cellSize };
    }
    function key(gx, gy) { return `${gx},${gy}`; }
    function heuristic(gx, gy) {
        const w = toWorld(gx, gy);
        return worldDist(w, goal);
    }

    const [sx, sy] = toGrid(start.x, start.y);
    const [gx, gy] = toGrid(goal.x, goal.y);

    // Open set as simple sorted list (good enough for demo)
    const open = [{ gx: sx, gy: sy, h: heuristic(sx, sy), parent: null }];
    const closed = new Set();
    const parentMap = new Map();
    const treeEdges = [];

    const dirs = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]];

    let found = false;
    let iterations = 0;

    while (open.length > 0 && iterations < 50000) {
        iterations++;
        // Pick smallest heuristic
        let bestIdx = 0;
        for (let i = 1; i < open.length; i++) {
            if (open[i].h < open[bestIdx].h) bestIdx = i;
        }
        const curr = open.splice(bestIdx, 1)[0];
        const ck = key(curr.gx, curr.gy);
        if (closed.has(ck)) continue;
        closed.add(ck);
        if (curr.parent !== null) parentMap.set(ck, curr.parent);

        // Record tree edge for visualization
        if (curr.parent) {
            treeEdges.push({ from: toWorld(curr.parent.gx, curr.parent.gy), to: toWorld(curr.gx, curr.gy) });
        }

        if (curr.gx === gx && curr.gy === gy) { found = true; break; }

        for (const [dx, dy] of dirs) {
            const nx = curr.gx + dx, ny = curr.gy + dy;
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
            const nk = key(nx, ny);
            if (closed.has(nk)) continue;
            const wp = toWorld(nx, ny);
            if (!pointFree(wp)) continue;
            open.push({ gx: nx, gy: ny, h: heuristic(nx, ny), parent: { gx: curr.gx, gy: curr.gy } });
        }
    }

    // Reconstruct path
    const path = [];
    if (found) {
        let ck = key(gx, gy);
        while (ck) {
            const [a, b] = ck.split(',').map(Number);
            path.unshift(toWorld(a, b));
            const p = parentMap.get(ck);
            ck = p ? key(p.gx, p.gy) : null;
        }
    }

    return { path, treeEdges, iterations };
}

// ── Planning dispatch ────────────────────────────────────────────────────────
function runPlanning() {
    if (demo.planning) return;
    if (!demo.start || !demo.goal) {
        setStatus('⚠ Place both Start (green) and Goal (red) first.');
        return;
    }

    demo.planning = true;
    demo.tree = [];
    demo.path = [];
    draw();

    const algo = document.getElementById('algo-select').value;
    const maxIters = parseInt(document.getElementById('max-iters').value);
    const stepSize = parseFloat(document.getElementById('step-size').value);
    const goalBias = parseFloat(document.getElementById('goal-bias').value);
    const animSpeed = parseInt(document.getElementById('anim-speed').value);

    document.getElementById('btn-plan').textContent = '⏳ Planning…';

    if (algo === 'rrt-star') {
        runRRTStarAnimated(maxIters, stepSize, goalBias, animSpeed);
    } else {
        runGreedyAnimated(animSpeed);
    }
}

function runRRTStarAnimated(maxIters, stepSize, goalBias, batchPerFrame) {
    demo.tree = [{ x: demo.start.x, y: demo.start.y, parent: -1, cost: 0 }];
    let iter = 0;
    const goalTol = stepSize * 1.2;
    const t0 = performance.now();

    function frame() {
        for (let b = 0; b < batchPerFrame && iter < maxIters; b++, iter++) {
            rrtStarStep(demo.tree, demo.goal, { stepSize, goalBias });
        }

        // Check if any node near goal
        let goalNodeIdx = -1;
        let bestGoalCost = Infinity;
        for (let i = 0; i < demo.tree.length; i++) {
            if (worldDist(demo.tree[i], demo.goal) < goalTol) {
                const c = demo.tree[i].cost + worldDist(demo.tree[i], demo.goal);
                if (c < bestGoalCost) { bestGoalCost = c; goalNodeIdx = i; }
            }
        }

        draw();
        setStatus(`RRT* — iteration ${iter} / ${maxIters} | tree: ${demo.tree.length} nodes`);
        updateDemoStats(demo.tree.length, null, performance.now() - t0, iter);

        if (iter >= maxIters || goalNodeIdx >= 0) {
            // Extract path
            if (goalNodeIdx >= 0) {
                demo.path = [{ x: demo.goal.x, y: demo.goal.y }];
                let idx = goalNodeIdx;
                while (idx >= 0) {
                    demo.path.unshift({ x: demo.tree[idx].x, y: demo.tree[idx].y });
                    idx = demo.tree[idx].parent;
                }
                const pathLen = computePathLength(demo.path);
                setStatus(`✅ RRT* — Path found! Length: ${pathLen.toFixed(2)} m | ${iter} iterations`);
                updateDemoStats(demo.tree.length, pathLen, performance.now() - t0, iter);
            } else {
                setStatus(`⚠ RRT* — No path found in ${maxIters} iterations.`);
            }
            draw();
            demo.planning = false;
            document.getElementById('btn-plan').textContent = '▶ Plan';
            return;
        }

        demo.animHandle = requestAnimationFrame(frame);
    }

    demo.animHandle = requestAnimationFrame(frame);
}

function runGreedyAnimated(batchPerFrame) {
    const t0 = performance.now();
    const result = greedyBestFirst(demo.start, demo.goal, demo.obstacles);
    const elapsed = performance.now() - t0;

    // Convert tree edges to our tree format for drawing
    demo.tree = [{ x: demo.start.x, y: demo.start.y, parent: -1 }];
    // Animate tree edge drawing
    let edgeIdx = 0;
    const edgesPerFrame = Math.max(1, batchPerFrame * 5);

    function frame() {
        for (let b = 0; b < edgesPerFrame && edgeIdx < result.treeEdges.length; b++, edgeIdx++) {
            const e = result.treeEdges[edgeIdx];
            const parentIdx = demo.tree.length - 1; // approximate for visual
            demo.tree.push({ x: e.to.x, y: e.to.y, parent: Math.max(0, parentIdx) });
        }

        draw();
        setStatus(`Greedy BFS — expanding ${edgeIdx} / ${result.treeEdges.length} nodes`);

        if (edgeIdx >= result.treeEdges.length) {
            demo.path = result.path;
            draw();
            const pathLen = result.path.length > 0 ? computePathLength(result.path) : 0;
            if (result.path.length > 0) {
                setStatus(`✅ Greedy BFS — Path found! Length: ${pathLen.toFixed(2)} m | ${result.iterations} cells explored`);
            } else {
                setStatus(`⚠ Greedy BFS — No path found.`);
            }
            updateDemoStats(result.treeEdges.length, pathLen, elapsed, result.iterations);
            demo.planning = false;
            document.getElementById('btn-plan').textContent = '▶ Plan';
            return;
        }

        demo.animHandle = requestAnimationFrame(frame);
    }

    demo.animHandle = requestAnimationFrame(frame);
}

function computePathLength(path) {
    let len = 0;
    for (let i = 1; i < path.length; i++) len += worldDist(path[i - 1], path[i]);
    return len;
}

// ── UI Helpers ───────────────────────────────────────────────────────────────
function setStatus(msg) {
    document.getElementById('demo-status').textContent = msg;
}

function updateDemoStats(nodes, pathLen, timeMs, iters) {
    document.getElementById('demo-stat-nodes').textContent = nodes ?? '—';
    document.getElementById('demo-stat-length').textContent = pathLen !== null ? pathLen.toFixed(2) : '—';
    document.getElementById('demo-stat-time').textContent = timeMs !== null ? timeMs.toFixed(1) : '—';
    document.getElementById('demo-stat-iters').textContent = iters ?? '—';
}

function clearAll() {
    if (demo.animHandle) cancelAnimationFrame(demo.animHandle);
    demo.planning = false;
    demo.start = null;
    demo.goal = null;
    demo.obstacles = [];
    demo.tree = [];
    demo.path = [];
    setStatus('Click the canvas to place Start, then Goal, then obstacles. Press ▶ Plan to run.');
    updateDemoStats(null, null, null, null);
    document.getElementById('btn-plan').textContent = '▶ Plan';
    draw();
}

// ── Presets ──────────────────────────────────────────────────────────────────
function presetNarrowGap() {
    clearAll();
    demo.start = { x: -7, y: 0 };
    demo.goal = { x: 7, y: 0 };
    // Vertical wall with narrow gap
    for (let y = -6; y <= 6; y += 1.2) {
        if (Math.abs(y) < 1.0) continue;  // gap at y≈0
        demo.obstacles.push({ cx: 0, cy: y, r: 0.7 });
    }
    setStatus('Preset: Narrow Gap — Press ▶ Plan to run.');
    draw();
}

function presetMaze() {
    clearAll();
    demo.start = { x: -8, y: -5 };
    demo.goal = { x: 8, y: 5 };

    // Horizontal walls
    const walls = [
        // Wall 1: left side
        { x0: -6, x1: 2, y: -3, r: 0.5, gap: null },
        // Wall 2: right side
        { x0: -2, x1: 6, y: 0, r: 0.5, gap: null },
        // Wall 3: mid
        { x0: -6, x1: 3, y: 3, r: 0.5, gap: null },
    ];
    for (const w of walls) {
        for (let x = w.x0; x <= w.x1; x += 1.1) {
            demo.obstacles.push({ cx: x, cy: w.y, r: w.r });
        }
    }
    // Vertical partial walls
    for (let y = -6; y <= -3; y += 1.1) demo.obstacles.push({ cx: 4, cy: y, r: 0.5 });
    for (let y = 0; y <= 6; y += 1.1) demo.obstacles.push({ cx: -4, cy: y, r: 0.5 });

    setStatus('Preset: Maze — Press ▶ Plan to run.');
    draw();
}

// ── Init ─────────────────────────────────────────────────────────────────────
function initDemo() {
    demo.canvas = document.getElementById('demo-canvas');
    demo.ctx = demo.canvas.getContext('2d');

    // Handle HiDPI
    const dpr = window.devicePixelRatio || 1;
    const rect = demo.canvas.getBoundingClientRect();
    demo.W = rect.width;
    demo.H = rect.width * (600 / 900);
    demo.canvas.width = demo.W * dpr;
    demo.canvas.height = demo.H * dpr;
    demo.canvas.style.height = demo.H + 'px';
    demo.ctx.scale(dpr, dpr);

    // Canvas click
    demo.canvas.addEventListener('click', (e) => {
        if (demo.planning) return;
        const rect = demo.canvas.getBoundingClientRect();
        const px = (e.clientX - rect.left) * (demo.W / rect.width);
        const py = (e.clientY - rect.top) * (demo.H / rect.height);
        const [wx, wy] = canvasToWorld(px, py);

        if (demo.mode === 'start') {
            demo.start = { x: wx, y: wy };
            demo.tree = [];
            demo.path = [];
            setStatus(`Start placed at (${wx.toFixed(1)}, ${wy.toFixed(1)}). Now set Goal or add obstacles.`);
        } else if (demo.mode === 'goal') {
            demo.goal = { x: wx, y: wy };
            demo.tree = [];
            demo.path = [];
            setStatus(`Goal placed at (${wx.toFixed(1)}, ${wy.toFixed(1)}). Add obstacles or press ▶ Plan.`);
        } else if (demo.mode === 'obstacle') {
            const r = parseFloat(document.getElementById('obs-radius').value);
            demo.obstacles.push({ cx: wx, cy: wy, r });
            demo.tree = [];
            demo.path = [];
            setStatus(`Obstacle added at (${wx.toFixed(1)}, ${wy.toFixed(1)}), r=${r}. Click more or ▶ Plan.`);
        }
        draw();
    });

    // Mode buttons
    document.querySelectorAll('#mode-group .btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('#mode-group .btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            demo.mode = btn.dataset.mode;
        });
    });

    // Range sliders
    function bindRange(id, displayId, fmt) {
        const el = document.getElementById(id);
        const disp = document.getElementById(displayId);
        el.addEventListener('input', () => { disp.textContent = fmt(el.value); });
    }
    bindRange('obs-radius', 'radius-val', v => parseFloat(v).toFixed(1));
    bindRange('max-iters', 'iter-val', v => v);
    bindRange('step-size', 'step-val', v => parseFloat(v).toFixed(1));
    bindRange('goal-bias', 'bias-val', v => parseFloat(v).toFixed(2));
    bindRange('anim-speed', 'speed-val', v => v);

    // Buttons
    document.getElementById('btn-plan').addEventListener('click', runPlanning);
    document.getElementById('btn-clear').addEventListener('click', clearAll);
    document.getElementById('btn-preset-1').addEventListener('click', presetNarrowGap);
    document.getElementById('btn-preset-2').addEventListener('click', presetMaze);

    // Keyboard shortcut
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') clearAll();
        if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); runPlanning(); }
    });

    // Resize handler
    window.addEventListener('resize', () => {
        const rect = demo.canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        demo.W = rect.width;
        demo.H = rect.width * (600 / 900);
        demo.canvas.width = demo.W * dpr;
        demo.canvas.height = demo.H * dpr;
        demo.canvas.style.height = demo.H + 'px';
        demo.ctx.setTransform(1, 0, 0, 1, 0, 0);
        demo.ctx.scale(dpr, dpr);
        draw();
    });

    draw();
}

// Boot when DOM ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initDemo);
} else {
    initDemo();
}
