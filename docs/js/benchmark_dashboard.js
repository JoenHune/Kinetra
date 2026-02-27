/**
 * Kinetra Benchmark Dashboard
 * Renders Google Benchmark JSON output files.
 */

function loadBenchmarks(data) {
    const benchmarks = data.benchmarks || [];
    if (benchmarks.length === 0) return;

    renderBenchTable(benchmarks);
    renderBenchChart(benchmarks);
}

function renderBenchTable(benchmarks) {
    const tbody = document.querySelector('#bench-table tbody');
    tbody.innerHTML = '';

    for (const b of benchmarks) {
        if (b.run_type === 'aggregate' && b.aggregate_name !== 'mean') continue;

        const row = document.createElement('tr');
        const name = b.name.replace(/_mean$/, '');
        const time = b.real_time !== undefined ? Math.round(b.real_time) : '—';
        const cpu = b.cpu_time !== undefined ? Math.round(b.cpu_time) : '—';
        const iters = b.iterations || '—';
        row.innerHTML = `<td>${name}</td><td>${time}</td><td>${cpu}</td><td>${iters}</td>`;
        tbody.appendChild(row);
    }
}

function renderBenchChart(benchmarks) {
    // Filter to non-aggregate entries
    const runs = benchmarks.filter(b => !b.run_type || b.run_type === 'iteration');
    if (runs.length === 0) return;

    // Group by benchmark name
    const groups = {};
    for (const b of runs) {
        const base = b.name.replace(/\/\d+$/, '');
        if (!groups[base]) groups[base] = { names: [], times: [] };
        groups[base].names.push(b.name);
        groups[base].times.push(b.real_time);
    }

    const traces = Object.entries(groups).map(([name, g]) => ({
        x: g.names,
        y: g.times,
        type: 'bar',
        name,
    }));

    Plotly.newPlot('plot-bench-time', traces, {
        paper_bgcolor: '#0d1117',
        plot_bgcolor: '#161b22',
        font: { color: '#c9d1d9' },
        margin: { l: 60, r: 20, t: 30, b: 120 },
        xaxis: { gridcolor: '#30363d', tickangle: -45 },
        yaxis: { gridcolor: '#30363d', title: 'Time (ns)' },
        barmode: 'group',
        title: { text: 'Benchmark Results', font: { size: 14, color: '#c9d1d9' } },
    }, { responsive: true });
}
