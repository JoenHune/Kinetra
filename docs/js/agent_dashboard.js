/**
 * Kinetra Agent Dashboard
 * Renders the agent iteration report (report.json).
 */

function loadAgentReport(data) {
    if (!Array.isArray(data) || data.length === 0) return;

    renderAgentTable(data);
    renderAgentTimeline(data);
}

function renderAgentTable(iterations) {
    const tbody = document.querySelector('#agent-table tbody');
    tbody.innerHTML = '';

    for (const rec of iterations) {
        const phases = {};
        for (const p of (rec.phases || [])) {
            phases[p.phase] = p;
        }

        const statusIcon = ok => ok ? '<span class="status-ok">✓</span>' : '<span class="status-fail">✗</span>';
        const phaseStatus = name => {
            const p = phases[name];
            return p ? statusIcon(p.success) : '—';
        };
        const totalTime = (rec.phases || []).reduce((s, p) => s + (p.duration_s || 0), 0);

        const row = document.createElement('tr');
        row.innerHTML = `
            <td>${rec.iteration}</td>
            <td>${rec.focus || ''}</td>
            <td>${phaseStatus('build')}</td>
            <td>${phaseStatus('test')}</td>
            <td>${phaseStatus('benchmark')}</td>
            <td>${totalTime.toFixed(1)}</td>
        `;
        tbody.appendChild(row);
    }
}

function renderAgentTimeline(iterations) {
    const phaseNames = ['research', 'build', 'test', 'benchmark', 'reflect'];
    const colors = ['#58a6ff', '#3fb950', '#d29922', '#f0883e', '#bc8cff'];

    const traces = phaseNames.map((pname, idx) => ({
        x: iterations.map(r => r.iteration),
        y: iterations.map(r => {
            const p = (r.phases || []).find(p => p.phase === pname);
            return p ? p.duration_s : 0;
        }),
        name: pname,
        type: 'bar',
        marker: { color: colors[idx] },
    }));

    Plotly.newPlot('plot-agent', traces, {
        paper_bgcolor: '#0d1117',
        plot_bgcolor: '#161b22',
        font: { color: '#c9d1d9' },
        margin: { l: 50, r: 20, t: 30, b: 50 },
        xaxis: { gridcolor: '#30363d', title: 'Iteration', dtick: 1 },
        yaxis: { gridcolor: '#30363d', title: 'Time (s)' },
        barmode: 'stack',
        title: { text: 'Agent Iteration Timeline', font: { size: 14, color: '#c9d1d9' } },
        legend: { orientation: 'h', y: -0.2 },
    }, { responsive: true });
}
