/**
 * Kinetra Benchmark Dashboard — Theme-aware
 * Displays benchmark results from Google Benchmark JSON output
 */

function getBenchTheme() {
    const dk = document.documentElement.getAttribute('data-theme') !== 'light';
    return {
        paper_bgcolor: dk ? '#0d1117' : '#ffffff',
        plot_bgcolor:  dk ? '#161b22' : '#f6f8fa',
        font:          { color: dk ? '#c9d1d9' : '#24292f', family: '-apple-system, sans-serif' },
        gridcolor:     dk ? '#30363d' : '#d0d7de',
    };
}

function loadBenchmarks(data) {
    renderBenchTable(data);
    renderBenchChart(data);
}

function renderBenchTable(data) {
    const el = document.getElementById('bench-table-body');
    if (!el) return; el.innerHTML = '';
    const bms = data.benchmarks || [];
    for (const bm of bms) {
        const row = document.createElement('tr');
        row.innerHTML = `
            <td>${bm.name || '-'}</td>
            <td>${bm.real_time != null ? bm.real_time.toFixed(1) : '-'}</td>
            <td>${bm.cpu_time != null ? bm.cpu_time.toFixed(1) : '-'}</td>
            <td>${bm.iterations || '-'}</td>
            <td>${bm.time_unit || 'ns'}</td>`;
        el.appendChild(row);
    }
}

function renderBenchChart(data) {
    const bms = data.benchmarks || []; if (bms.length === 0) return;
    const th = getBenchTheme();
    Plotly.newPlot('bench-chart', [{
        x: bms.map(b => b.name),
        y: bms.map(b => b.real_time || 0),
        type: 'bar',
        marker: { color: '#58a6ff' },
        name: 'Real Time',
    }, {
        x: bms.map(b => b.name),
        y: bms.map(b => b.cpu_time || 0),
        type: 'bar',
        marker: { color: '#3fb950' },
        name: 'CPU Time',
    }], {
        paper_bgcolor: th.paper_bgcolor, plot_bgcolor: th.plot_bgcolor, font: th.font,
        barmode: 'group',
        margin: { l: 60, r: 20, t: 30, b: 120 },
        xaxis: { gridcolor: th.gridcolor, tickangle: -30 },
        yaxis: { gridcolor: th.gridcolor, title: 'Time (' + (bms[0]?.time_unit || 'ns') + ')' },
        title: { text: 'Benchmark Results', font: { size: 14 } },
    }, { responsive: true });
}
