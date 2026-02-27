/**
 * Kinetra Trajectory Viewer — Theme-aware
 * Renders planning results exported by kinetra::io::generateVisualizationHTML()
 */

function getViewerTheme() {
    const dk = document.documentElement.getAttribute('data-theme') !== 'light';
    return {
        paper_bgcolor: dk ? '#0d1117' : '#ffffff',
        plot_bgcolor:  dk ? '#161b22' : '#f6f8fa',
        font:          { color: dk ? '#c9d1d9' : '#24292f', family: '-apple-system, sans-serif' },
        gridcolor:     dk ? '#30363d' : '#d0d7de',
        zerolinecolor: dk ? '#30363d' : '#d0d7de',
    };
}

function loadTrajectory(data) {
    renderPathPlot(data);
    renderViewerVelocityPlot(data);
    updatePathStats(data);
}

function renderPathPlot(data) {
    const traces = [];
    const th = getViewerTheme();

    if (data.environment && data.environment.obstacles) {
        for (const obs of data.environment.obstacles) {
            if (obs.type === 'circle') {
                const theta = Array.from({ length: 64 }, (_, i) => (i / 63) * 2 * Math.PI);
                traces.push({
                    x: theta.map(t => obs.cx + obs.radius * Math.cos(t)),
                    y: theta.map(t => obs.cy + obs.radius * Math.sin(t)),
                    mode: 'lines', fill: 'toself',
                    fillcolor: 'rgba(248,81,73,0.3)', line: { color: '#f85149', width: 1 },
                    name: 'Obstacle', showlegend: traces.length === 0,
                });
            } else if (obs.type === 'rectangle') {
                traces.push({
                    x: [obs.min_x, obs.max_x, obs.max_x, obs.min_x, obs.min_x],
                    y: [obs.min_y, obs.min_y, obs.max_y, obs.max_y, obs.min_y],
                    mode: 'lines', fill: 'toself',
                    fillcolor: 'rgba(248,81,73,0.3)', line: { color: '#f85149', width: 1 },
                    name: 'Obstacle', showlegend: false,
                });
            }
        }
    }

    if (data.trajectory) {
        const wp = data.trajectory;
        traces.push({
            x: wp.map(p => p.x), y: wp.map(p => p.y),
            mode: 'lines+markers',
            line: { color: '#58a6ff', width: 2 },
            marker: { size: 4, color: '#58a6ff' },
            name: 'Path',
        });
        for (let i = 0; i < wp.length; i += Math.max(1, Math.floor(wp.length / 20))) {
            const p = wp[i];
            if (p.theta !== undefined) {
                traces.push({
                    x: [p.x, p.x + 0.3 * Math.cos(p.theta)],
                    y: [p.y, p.y + 0.3 * Math.sin(p.theta)],
                    mode: 'lines', line: { color: '#3fb950', width: 1.5 }, showlegend: false,
                });
            }
        }
    }

    if (data.start) traces.push({ x: [data.start.x||data.start[0]], y: [data.start.y||data.start[1]], mode:'markers', marker:{size:12,color:'#3fb950',symbol:'diamond'}, name:'Start' });
    if (data.goal)  traces.push({ x: [data.goal.x||data.goal[0]], y: [data.goal.y||data.goal[1]], mode:'markers', marker:{size:12,color:'#f85149',symbol:'star'}, name:'Goal' });

    Plotly.newPlot('plot-path', traces, {
        paper_bgcolor: th.paper_bgcolor, plot_bgcolor: th.plot_bgcolor,
        font: th.font, margin: { l: 50, r: 20, t: 30, b: 50 },
        xaxis: { gridcolor: th.gridcolor, zerolinecolor: th.zerolinecolor, title: 'X (m)' },
        yaxis: { gridcolor: th.gridcolor, zerolinecolor: th.zerolinecolor, title: 'Y (m)', scaleanchor: 'x' },
        title: { text: 'Planning Result', font: { size: 14 } },
    }, { responsive: true });
}

function renderViewerVelocityPlot(data) {
    if (!data.trajectory || data.trajectory.length < 2) return;
    const wp = data.trajectory, times = [], vels = [];
    for (let i = 1; i < wp.length; i++) {
        const dx = wp[i].x - wp[i-1].x, dy = wp[i].y - wp[i-1].y;
        const dt = (wp[i].t || i * 0.1) - (wp[i-1].t || (i-1) * 0.1);
        times.push(wp[i].t || i * 0.1);
        vels.push(dt > 0 ? Math.sqrt(dx*dx + dy*dy) / dt : 0);
    }
    const th = getViewerTheme();
    Plotly.newPlot('plot-velocity', [{
        x: times, y: vels, mode: 'lines',
        line: { color: '#d29922', width: 2 },
        fill: 'tozeroy', fillcolor: 'rgba(210,153,34,0.15)', name: 'Velocity',
    }], {
        paper_bgcolor: th.paper_bgcolor, plot_bgcolor: th.plot_bgcolor, font: th.font,
        margin: { l: 50, r: 20, t: 30, b: 50 },
        xaxis: { gridcolor: th.gridcolor, title: 'Time (s)' },
        yaxis: { gridcolor: th.gridcolor, title: 'Velocity (m/s)' },
        title: { text: 'Velocity Profile', font: { size: 14 } },
    }, { responsive: true });
}

function updatePathStats(data) {
    document.getElementById('stat-length').textContent = data.path_length != null ? data.path_length.toFixed(2) : '-';
    document.getElementById('stat-time').textContent = data.solve_time_ms != null ? data.solve_time_ms.toFixed(1) : '-';
    document.getElementById('stat-smoothness').textContent = data.smoothness != null ? data.smoothness.toFixed(4) : '-';
    document.getElementById('stat-curvature').textContent = data.max_curvature != null ? data.max_curvature.toFixed(3) : '-';
    document.getElementById('stat-waypoints').textContent = data.trajectory ? data.trajectory.length : '-';
}
