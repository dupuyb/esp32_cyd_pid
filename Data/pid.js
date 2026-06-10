(function () {
  const wsStatus = document.getElementById('wsStatus');
  const currentTempEl = document.getElementById('currentTemp');
  const currentSetpointEl = document.getElementById('currentSetpoint');
  const humidityEl = document.getElementById('humidity');
  const pwmEl = document.getElementById('pwm');
  const pidStateEl = document.getElementById('pidState');
  const wsPointCountEl = document.getElementById('wsPointCount');

  const setpointInput = document.getElementById('setpoint');
  const kpInput = document.getElementById('kp');
  const kiInput = document.getElementById('ki');
  const kdInput = document.getElementById('kd');
  const pidEnabledInput = document.getElementById('pidEnabled');
  const vmcManualOnInput = document.getElementById('vmcManualOn');
  const applyBtn = document.getElementById('applyBtn');
  const refreshBtn = document.getElementById('refreshBtn');

  const canvas = document.getElementById('chart');
  const ctx = canvas.getContext('2d');

  let ws = null;
  let historyTemp = [];
  let historyPwm = [];
  let historyPoints = 0;
  let wsPointCount = 0;
  let syncInputsFromServer = true;
  let hasLocalDraft = false;

  function updatePointCounter() {
    if (wsPointCountEl) {
      wsPointCountEl.textContent = String(wsPointCount);
    }
  }

  const formInputs = [setpointInput, kpInput, kiInput, kdInput, pidEnabledInput, vmcManualOnInput];
  formInputs.forEach(function (input) {
    input.addEventListener('input', function () {
      hasLocalDraft = true;
    });
    input.addEventListener('change', function () {
      hasLocalDraft = true;
    });
  });

  function fmt(value, decimals, suffix) {
    if (!Number.isFinite(value)) {
      return '--' + (suffix ? ' ' + suffix : '');
    }
    return value.toFixed(decimals) + (suffix ? ' ' + suffix : '');
  }

  function appendHistoryPoint(temp, pwm) {
    const maxLen = historyPoints || 60;
    if (Number.isFinite(temp)) {
      historyTemp.push(temp);
      if (historyTemp.length > maxLen) historyTemp.splice(0, historyTemp.length - maxLen);
    }
    if (Number.isFinite(pwm)) {
      historyPwm.push(pwm);
      if (historyPwm.length > maxLen) historyPwm.splice(0, historyPwm.length - maxLen);
    }
  }

  function drawChart() {
    const width = canvas.width;
    const height = canvas.height;
    const padLeft = 46;
    const padRight = 16;
    const padTop = 16;
    const padBottom = 28;

    ctx.clearRect(0, 0, width, height);

    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = padTop + ((height - padTop - padBottom) * i) / 4;
      ctx.beginPath();
      ctx.moveTo(padLeft, y);
      ctx.lineTo(width - padRight, y);
      ctx.stroke();
    }

    ctx.strokeStyle = '#94a3b8';
    ctx.beginPath();
    ctx.moveTo(padLeft, padTop);
    ctx.lineTo(padLeft, height - padBottom);
    ctx.lineTo(width - padRight, height - padBottom);
    ctx.stroke();

    const points = Math.max(historyPoints || 0, historyTemp.length, historyPwm.length);

    function seriesValueAt(series, i) {
      if (points <= 0) {
        return Number.NaN;
      }
      // Right-align samples so newest value stays on the right edge.
      const start = points - series.length;
      if (i < start) {
        return Number.NaN;
      }
      return series[i - start];
    }

    if (points < 2 || historyTemp.length < 2 || historyPwm.length < 2) {
      ctx.fillStyle = '#64748b';
      ctx.font = '14px Trebuchet MS, sans-serif';
      ctx.fillText('Waiting for history data...', padLeft + 10, height / 2);
      return;
    }

    let minT = Number.POSITIVE_INFINITY;
    let maxT = Number.NEGATIVE_INFINITY;
    for (let i = 0; i < points; i++) {
      const t = seriesValueAt(historyTemp, i);
      if (Number.isFinite(t)) {
        if (t < minT) minT = t;
        if (t > maxT) maxT = t;
      }
    }
    if (!Number.isFinite(minT) || !Number.isFinite(maxT)) {
      minT = 20;
      maxT = 40;
    }

    if (Math.abs(maxT - minT) < 0.2) {
      minT -= 0.5;
      maxT += 0.5;
    }

    minT = Math.floor(minT - 0.5);
    maxT = Math.ceil(maxT + 0.5);

    function xOf(i) {
      return padLeft + (i * (width - padLeft - padRight)) / (points - 1);
    }

    function yTemp(v) {
      return padTop + ((maxT - v) * (height - padTop - padBottom)) / (maxT - minT);
    }

    function yPwm(v) {
      return padTop + ((100 - v) * (height - padTop - padBottom)) / 100;
    }

    ctx.fillStyle = '#475569';
    ctx.font = '11px Trebuchet MS, sans-serif';
    ctx.fillText(maxT.toFixed(1) + ' C', 4, padTop + 8);
    ctx.fillText(minT.toFixed(1) + ' C', 4, height - padBottom);
    ctx.fillText('100%', width - 44, padTop + 8);
    ctx.fillText('0%', width - 28, height - padBottom);

    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    let tempLineStarted = false;
    for (let i = 0; i < points; i++) {
      const x = xOf(i);
      const value = seriesValueAt(historyTemp, i);
      if (!Number.isFinite(value)) {
        tempLineStarted = false;
        continue;
      }
      const y = yTemp(value);
      if (!tempLineStarted) {
        ctx.moveTo(x, y);
        tempLineStarted = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();

    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 2;
    ctx.beginPath();
    let pwmLineStarted = false;
    for (let i = 0; i < points; i++) {
      const x = xOf(i);
      const value = seriesValueAt(historyPwm, i);
      if (!Number.isFinite(value)) {
        pwmLineStarted = false;
        continue;
      }
      const y = yPwm(value);
      if (!pwmLineStarted) {
        ctx.moveTo(x, y);
        pwmLineStarted = true;
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }

  function updateState(data) {
    if (Number.isFinite(data.temperature)) {
      currentTempEl.textContent = fmt(data.temperature, 1, 'C');
    }
    const shouldSyncInputs = syncInputsFromServer && !hasLocalDraft;

    if (Number.isFinite(data.setpoint)) {
      currentSetpointEl.textContent = fmt(data.setpoint, 1, 'C');
      if (shouldSyncInputs) {
        setpointInput.value = data.setpoint.toFixed(1);
      }
    }
    if (data.humidity !== undefined) {
      humidityEl.textContent = data.humidity + ' %';
    }
    if (Number.isFinite(data.pwm)) {
      pwmEl.textContent = fmt(data.pwm, 0, '%');
    }

    if (Number.isFinite(data.historyPoints)) {
      historyPoints = Math.max(0, Math.floor(data.historyPoints));
    }

    if (typeof data.pidEnabled === 'boolean') {
      if (shouldSyncInputs) {
        pidEnabledInput.checked = data.pidEnabled;
      }
      pidStateEl.textContent = data.pidEnabled ? 'ON' : 'OFF';
      pidStateEl.className = data.pidEnabled ? 'on' : 'off';
    }

    if (typeof data.vmcManualOn === 'boolean') {
      if (shouldSyncInputs) {
        vmcManualOnInput.checked = data.vmcManualOn;
      }
    }

    if (shouldSyncInputs) {
      if (Number.isFinite(data.kp)) kpInput.value = data.kp.toFixed(3);
      if (Number.isFinite(data.ki)) kiInput.value = data.ki.toFixed(3);
      if (Number.isFinite(data.kd)) kdInput.value = data.kd.toFixed(3);
    }

    if (Number.isFinite(data.temperature) || Number.isFinite(data.pwm)) {
      appendHistoryPoint(data.temperature, data.pwm);
      drawChart();
    }

    if (shouldSyncInputs) {
      syncInputsFromServer = false;
    }
  }

  function sendJson(payload) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return;
    }
    ws.send(JSON.stringify(payload));
  }

  function connect() {
    ws = new WebSocket('ws://' + window.location.hostname + ':81/', ['arduino']);

    ws.onopen = function () {
      wsStatus.textContent = 'WebSocket: connected';
      wsPointCount = 0;
      updatePointCounter();
      syncInputsFromServer = true;
      sendJson({ request: 'full' });
    };

    ws.onerror = function () {
      wsStatus.textContent = 'WebSocket: error';
    };

    ws.onclose = function () {
      wsStatus.textContent = 'WebSocket: disconnected, retrying...';
      setTimeout(connect, 2000);
    };

    ws.onmessage = function (evt) {
      let data = null;
      try {
        data = JSON.parse(evt.data);
      } catch (e) {
        return;
      }
      if (!data || !data.type) {
        return;
      }
      if (data.type === 'pid_state') {
        wsPointCount += 1;
        updatePointCounter();
        updateState(data);
        return;
      }
      if (data.type === 'error') {
        wsStatus.textContent = 'WebSocket: ' + data.message;
      }
    };
  }

  applyBtn.addEventListener('click', function () {
    sendJson({
      setpoint: parseFloat(setpointInput.value),
      kp: parseFloat(kpInput.value),
      ki: parseFloat(kiInput.value),
      kd: parseFloat(kdInput.value),
      pidEnabled: !!pidEnabledInput.checked,
      vmcManualOn: !!vmcManualOnInput.checked,
      request: 'history'
    });
    hasLocalDraft = false;
  });

  refreshBtn.addEventListener('click', function () {
    syncInputsFromServer = true;
    hasLocalDraft = false;
    sendJson({ request: 'full' });
  });

  window.addEventListener('resize', drawChart);

  connect();
})();
