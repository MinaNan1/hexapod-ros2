// main.js

const ROS_IP = window.location.hostname; // Assumes rosbridge is on the same host
const ROS_PORT = '9090'; // Default rosbridge websocket port

// DOM Elements
const statusIndicator = document.getElementById('ros-status');
const statusText = document.getElementById('ros-status-text');
const btnManual = document.getElementById('btn-manual');
const btnAuto = document.getElementById('btn-auto');
const btnFwd = document.getElementById('btn-fwd');
const btnBwd = document.getElementById('btn-bwd');
const btnStop = document.getElementById('btn-stop');
const valLinearY = document.getElementById('val-linear-y');
const valAngularZ = document.getElementById('val-angular-z');
const valState = document.getElementById('val-state');
const logConsole = document.getElementById('log-console');

// Slider DOM Elements
const sliders = {
    roll: { input: document.getElementById('slider-roll'), val: document.getElementById('val-roll') },
    pitch: { input: document.getElementById('slider-pitch'), val: document.getElementById('val-pitch') },
    yaw: { input: document.getElementById('slider-yaw'), val: document.getElementById('val-yaw') },
    tz: { input: document.getElementById('slider-tz'), val: document.getElementById('val-tz') },
};
const btnResetPose = document.getElementById('btn-reset-pose');

let manualMode = true;
let isJoystickWalking = false;
let currentWalkDir = 'stop';

// State
let currentMotion = { speedIntensity: 0, steering: 0 };
let currentPosture = { roll: 0, pitch: 0, yaw: 0, tz: 0 };

// Helper to add logs
function log(msg, level = 'info') {
    const el = document.createElement('div');
    el.className = `log-entry ${level}`;
    const time = new Date().toLocaleTimeString();
    el.textContent = `[${time}] ${msg}`;
    logConsole.appendChild(el);
    logConsole.scrollTop = logConsole.scrollHeight;
}

// ── 1. Setup ROS Connection ──────────────────────────────────────────────────
const ros = new ROSLIB.Ros({
    url: `ws://${ROS_IP}:${ROS_PORT}`
});

ros.on('connection', function () {
    statusIndicator.classList.add('connected');
    statusText.textContent = 'Connected to ROS';
    log('Successfully connected to websocket server.', 'success');
});

ros.on('error', function (error) {
    statusIndicator.classList.remove('connected');
    statusText.textContent = 'Error connecting to ROS';
    log(`Error connecting to websocket server: ${error}`, 'error');
});

ros.on('close', function () {
    statusIndicator.classList.remove('connected');
    statusText.textContent = 'Connection closed';
    log('Connection to websocket server closed.', 'warn');
});

// ── 2. Setup Publishers ──────────────────────────────────────────────────────
const cmdPub = new ROSLIB.Topic({
    ros: ros,
    name: '/ui/cmd',
    messageType: 'std_msgs/String'
});

const posePub = new ROSLIB.Topic({
    ros: ros,
    name: '/ui/body_pose',
    messageType: 'geometry_msgs/Twist'
});

const posturePub = new ROSLIB.Topic({
    ros: ros,
    name: '/ui/body_posture',
    messageType: 'geometry_msgs/Twist'
});

const modePub = new ROSLIB.Topic({
    ros: ros,
    name: '/ui/mode',
    messageType: 'std_msgs/Bool'
});

// Send a simple string command
function sendCmd(command) {
    if (!manualMode && command !== 'stop') {
        log('Cannot send command. Switch to MANUAL mode first.', 'warn');
        return;
    }
    const msg = new ROSLIB.Message({ data: command });
    cmdPub.publish(msg);
    log(`Sent command: ${command}`);
}

// Send an emote command — allowed regardless of manual/auto mode
function sendEmoteCmd(command) {
    const msg = new ROSLIB.Message({ data: command });
    cmdPub.publish(msg);
    log(`Sent emote command: ${command}`, 'info');
}

// ── Publish motion data (joystick speed + steering only) ─────────────────────
function publishMotion() {
    const twistMsg = new ROSLIB.Message({
        linear: { x: currentMotion.speedIntensity, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: currentMotion.steering * Math.PI / 180.0 }
    });
    posePub.publish(twistMsg);
}

// ── Publish posture data (sliders only, separate topic) ──────────────────────
function publishPosture() {
    const twistMsg = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: currentPosture.tz },
        angular: {
            x: currentPosture.roll * Math.PI / 180.0,
            y: currentPosture.pitch * Math.PI / 180.0,
            z: currentPosture.yaw * Math.PI / 180.0
        }
    });
    posturePub.publish(twistMsg);
}

// Publish mode toggle
function setMode(isManual) {
    manualMode = isManual;
    btnManual.classList.toggle('active', manualMode);
    btnAuto.classList.toggle('active', !manualMode);

    const msg = new ROSLIB.Message({ data: manualMode });
    modePub.publish(msg);
    log(`Switched to ${manualMode ? 'MANUAL' : 'AUTO'} mode`);

    // Cancel emotes on mode switch
    if (trumpEmoteActive) {
        stopTrumpEmote();
    }
    if (saluteEmoteActive) {
        stopSaluteEmote();
    }
}

// Button listeners
btnManual.addEventListener('click', () => setMode(true));
btnAuto.addEventListener('click', () => setMode(false));
btnFwd.addEventListener('click', () => sendCmd('walk'));
btnBwd.addEventListener('click', () => sendCmd('backward'));
btnStop.addEventListener('click', () => {
    // Also cancel any joint-level emote on hard stop
    if (trumpEmoteActive) stopTrumpEmote();
    if (saluteEmoteActive) stopSaluteEmote();
    sendCmd('stop');
    valState.textContent = 'STOPPED';
    valState.style.color = 'var(--danger)';
});

// ── 2.5 Setup Sliders ─────────────────────────────────────────────────────────

function updateSliders() {
    currentPosture.roll = parseFloat(sliders.roll.input.value);
    currentPosture.pitch = parseFloat(sliders.pitch.input.value);
    currentPosture.yaw = parseFloat(sliders.yaw.input.value);
    currentPosture.tz = parseFloat(sliders.tz.input.value);

    sliders.roll.val.textContent = currentPosture.roll.toFixed(1) + '°';
    sliders.pitch.val.textContent = currentPosture.pitch.toFixed(1) + '°';
    sliders.yaw.val.textContent = currentPosture.yaw.toFixed(1) + '°';
    sliders.tz.val.textContent = currentPosture.tz.toFixed(1) + ' mm';

    publishPosture();
}

Object.values(sliders).forEach(s => {
    s.input.addEventListener('input', updateSliders);
});

btnResetPose.addEventListener('click', () => {
    stopEmote();
    Object.values(sliders).forEach(s => { s.input.value = 0; });
    updateSliders();
});

// ── 2.6 Pose-Based Emote System (wave, dance, bow, etc.) ─────────────────────

const emoteStatus = document.getElementById('emote-status');
let emoteInterval = null;
let emoteTimeoutIds = [];

const EMOTES = {
    wave: {
        name: 'Wave',
        loop: false,
        keyframes: [
            { pose: { roll: 8, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
            { pose: { roll: -8, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
            { pose: { roll: 8, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
            { pose: { roll: -8, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
            { pose: { roll: 8, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
            { pose: { roll: -8, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 0 }, duration: 400 },
        ]
    },
    dance: {
        name: 'Dance',
        loop: false,
        keyframes: [
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: -10 }, duration: 200 },
            { pose: { roll: 6, pitch: -4, yaw: 10, tz: 5 }, duration: 300 },
            { pose: { roll: -6, pitch: 4, yaw: -10, tz: -5 }, duration: 300 },
            { pose: { roll: 6, pitch: 4, yaw: 10, tz: 5 }, duration: 300 },
            { pose: { roll: -6, pitch: -4, yaw: -10, tz: -5 }, duration: 300 },
            { pose: { roll: 8, pitch: 0, yaw: 15, tz: 8 }, duration: 300 },
            { pose: { roll: -8, pitch: 0, yaw: -15, tz: -8 }, duration: 300 },
            { pose: { roll: 6, pitch: -4, yaw: 10, tz: 5 }, duration: 300 },
            { pose: { roll: -6, pitch: 4, yaw: -10, tz: -5 }, duration: 300 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 0 }, duration: 400 },
        ]
    },
    bow: {
        name: 'Bow',
        loop: false,
        keyframes: [
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 5 }, duration: 300 },
            { pose: { roll: 0, pitch: 10, yaw: 0, tz: 0 }, duration: 500 },
            { pose: { roll: 0, pitch: 10, yaw: 0, tz: 0 }, duration: 800 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 5 }, duration: 500 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
        ]
    },
    stretch: {
        name: 'Stretch',
        loop: false,
        keyframes: [
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 15 }, duration: 600 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 15 }, duration: 400 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: -15 }, duration: 500 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: -15 }, duration: 300 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 10 }, duration: 400 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 0 }, duration: 400 },
        ]
    },
    wiggle: {
        name: 'Wiggle',
        loop: false,
        keyframes: [
            { pose: { roll: 0, pitch: 0, yaw: 15, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: -15, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: 15, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: -15, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: 10, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: -10, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: 5, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: -5, tz: 0 }, duration: 150 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 0 }, duration: 200 },
        ]
    },
    excited: {
        name: 'Excited',
        loop: false,
        keyframes: [
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 12 }, duration: 120 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: -5 }, duration: 120 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 12 }, duration: 120 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: -5 }, duration: 120 },
            { pose: { roll: 3, pitch: 0, yaw: 8, tz: 12 }, duration: 120 },
            { pose: { roll: -3, pitch: 0, yaw: -8, tz: -5 }, duration: 120 },
            { pose: { roll: 3, pitch: 0, yaw: 8, tz: 12 }, duration: 120 },
            { pose: { roll: -3, pitch: 0, yaw: -8, tz: -5 }, duration: 120 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 10 }, duration: 120 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: -3 }, duration: 120 },
            { pose: { roll: 0, pitch: 0, yaw: 0, tz: 0 }, duration: 300 },
        ]
    },
};

function setPosture(pose) {
    sliders.roll.input.value = pose.roll;
    sliders.pitch.input.value = pose.pitch;
    sliders.yaw.input.value = pose.yaw;
    sliders.tz.input.value = pose.tz;
    updateSliders();
}

function lerpPose(from, to, t) {
    return {
        roll: from.roll + (to.roll - from.roll) * t,
        pitch: from.pitch + (to.pitch - from.pitch) * t,
        yaw: from.yaw + (to.yaw - from.yaw) * t,
        tz: from.tz + (to.tz - from.tz) * t,
    };
}

function stopEmote() {
    if (emoteInterval) {
        cancelAnimationFrame(emoteInterval);
        emoteInterval = null;
    }
    emoteTimeoutIds.forEach(id => clearTimeout(id));
    emoteTimeoutIds = [];
    document.querySelectorAll('.emote-btn').forEach(b => b.classList.remove('playing'));
    emoteStatus.textContent = '';
}

function playEmote(emoteName) {
    const emote = EMOTES[emoteName];
    if (!emote) return;

    stopEmote();

    const btn = document.querySelector(`[data-emote="${emoteName}"]`);
    if (btn) btn.classList.add('playing');
    emoteStatus.textContent = `Playing: ${emote.name}...`;
    log(`Emote: ${emote.name}`, 'info');

    const keyframes = emote.keyframes;
    let currentKeyframe = 0;
    let startPose = { roll: 0, pitch: 0, yaw: 0, tz: 0 };
    let startTime = performance.now();

    function animate(now) {
        if (currentKeyframe >= keyframes.length) {
            stopEmote();
            return;
        }

        const kf = keyframes[currentKeyframe];
        const elapsed = now - startTime;
        const t = Math.min(1.0, elapsed / kf.duration);

        const eased = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;

        const interpolated = lerpPose(startPose, kf.pose, eased);
        setPosture(interpolated);

        if (t >= 1.0) {
            startPose = { ...kf.pose };
            currentKeyframe++;
            startTime = now;
        }

        emoteInterval = requestAnimationFrame(animate);
    }

    emoteInterval = requestAnimationFrame(animate);
}

document.querySelectorAll('.emote-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        playEmote(btn.dataset.emote);
    });
});

// ── 2.7 Trump Emote (Joint-Level — separate from pose emotes) ─────────────────

let trumpEmoteActive = false;
const btnTrump = document.getElementById('btn-trump');

function startTrumpEmote() {
    if (trumpEmoteActive) return;
    stopEmote();
    stopSaluteEmote();  // cancel salute if running
    trumpEmoteActive = true;
    btnTrump.classList.add('playing');
    emoteStatus.textContent = 'Playing: Trump Elbow Dance...';
    log('Trump emote: STARTED', 'info');
    sendEmoteCmd('trump');
}

function stopTrumpEmote() {
    if (!trumpEmoteActive) return;
    trumpEmoteActive = false;
    btnTrump.classList.remove('playing');
    emoteStatus.textContent = '';
    log('Trump emote: STOPPED', 'info');
    sendEmoteCmd('trump_stop');
}

function toggleTrumpEmote() {
    if (trumpEmoteActive) { stopTrumpEmote(); } else { startTrumpEmote(); }
}

if (btnTrump) {
    btnTrump.addEventListener('click', toggleTrumpEmote);
}

// ── 2.8 Salute Emote (Joint-Level) ───────────────────────────────────────────

let saluteEmoteActive = false;
const btnSalute = document.getElementById('btn-salute');

function startSaluteEmote() {
    if (saluteEmoteActive) return;
    stopEmote();
    stopTrumpEmote();  // cancel trump if running
    saluteEmoteActive = true;
    if (btnSalute) btnSalute.classList.add('playing');
    emoteStatus.textContent = 'Playing: Salute...';
    log('Salute emote: STARTED', 'info');
    sendEmoteCmd('salute');
}

function stopSaluteEmote() {
    if (!saluteEmoteActive) return;
    saluteEmoteActive = false;
    if (btnSalute) btnSalute.classList.remove('playing');
    emoteStatus.textContent = '';
    log('Salute emote: STOPPED', 'info');
    sendEmoteCmd('salute_stop');
}

function toggleSaluteEmote() {
    if (saluteEmoteActive) { stopSaluteEmote(); } else { startSaluteEmote(); }
}

if (btnSalute) {
    btnSalute.addEventListener('click', toggleSaluteEmote);
}

// ── 2.9 Gamepad Emote Cycling ────────────────────────────────────────────────
const EMOTE_LIST = ['wave', 'dance', 'bow', 'stretch', 'wiggle', 'excited', 'trump', 'salute'];
const EMOTE_LABELS = ['👋 Wave', '💃 Dance', '🎩 Bow', '🙆 Stretch', '🐛 Wiggle', '🎉 Excited', '🕺 Trump', '🫡 Salute'];
let selectedEmoteIndex = 6; // default to Trump

function selectEmote(direction) {
    selectedEmoteIndex = ((selectedEmoteIndex + direction) % EMOTE_LIST.length + EMOTE_LIST.length) % EMOTE_LIST.length;
    const emoteName = EMOTE_LIST[selectedEmoteIndex];
    const emoteLabel = EMOTE_LABELS[selectedEmoteIndex];

    // Highlight the selected emote button in the UI
    document.querySelectorAll('.emote-btn').forEach(b => {
        b.style.outline = '';
        b.style.outlineOffset = '';
    });
    const btn = document.querySelector(`[data-emote="${emoteName}"]`);
    if (btn) {
        btn.style.outline = '2px solid #00f0ff';
        btn.style.outlineOffset = '3px';
    }
    emoteStatus.textContent = `🎮 Selected: ${emoteLabel}`;
    log(`Gamepad: Selected emote → ${emoteLabel}`, 'info');
}

function playSelectedEmote() {
    const emoteName = EMOTE_LIST[selectedEmoteIndex];
    if (emoteName === 'trump') {
        toggleTrumpEmote();
    } else if (emoteName === 'salute') {
        toggleSaluteEmote();
    } else {
        // Pose-based emote — cancel any joint emotes first
        if (trumpEmoteActive) stopTrumpEmote();
        if (saluteEmoteActive) stopSaluteEmote();
        playEmote(emoteName);
    }
}

// ── 3. Setup Joystick ────────────────────────────────────────────────────────
const joystickZone = document.getElementById('joystick-zone');
const joystick = nipplejs.create({
    zone: joystickZone,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#00f0ff',
    size: 250
});

const MAX_LINEAR_Y = 50.0;
const MAX_ANGULAR_Z = 45.0;

function handleJoystick(data) {
    if (!manualMode) return;

    // Moving the joystick cancels any joint-level emote
    if ((trumpEmoteActive || saluteEmoteActive) && data && data.distance > 5) {
        stopTrumpEmote();
        stopSaluteEmote();
    }

    let forwardBackwardMagnitude = 0;
    currentMotion.speedIntensity = 0;
    currentMotion.steering = 0;

    if (data && data.angle && data.distance) {
        const intensity = Math.min(1.0, data.distance / 125.0);
        const angleRad = data.angle.radian;

        forwardBackwardMagnitude = intensity * Math.sin(angleRad);
        currentMotion.steering = -intensity * Math.cos(angleRad) * MAX_ANGULAR_Z;
        currentMotion.speedIntensity = intensity;
    }

    let targetDir = 'stop';
    if (forwardBackwardMagnitude > 0.1) targetDir = 'walk';
    else if (forwardBackwardMagnitude < -0.1) targetDir = 'backward';

    currentWalkDir = targetDir;
    isJoystickWalking = (targetDir !== 'stop');

    valLinearY.textContent = forwardBackwardMagnitude.toFixed(2);
    valAngularZ.textContent = currentMotion.steering.toFixed(2);

    if (isJoystickWalking || Math.abs(currentMotion.steering) > 0.1) {
        valState.textContent = 'MOVING';
        valState.style.color = 'var(--accent)';
    } else {
        valState.textContent = 'IDLE';
        valState.style.color = 'var(--text-main)';
    }
}

// ── 3.5 Continuous Publisher Loop (10Hz) ──────────────────────────────────────
setInterval(() => {
    if (!manualMode) return;
    publishMotion();
    if (isJoystickWalking) {
        sendCmd(currentWalkDir);
    }
}, 100);

joystick.on('move', (evt, data) => {
    handleJoystick(data);
});

joystick.on('end', () => {
    handleJoystick(null);
    sendCmd('stop');  // auto-stabilize: stop walking and return to standing pose
    valState.textContent = 'IDLE';
    valState.style.color = 'var(--text-main)';
});

// ── 4. Gamepad (PS3 / PS4 / Xbox) Support ────────────────────────────────────
const gamepadStatusIndicator = document.getElementById('gamepad-status');
const gamepadStatusText = document.getElementById('gamepad-status-text');

let gamepadIndex = null;
let gamepadActive = false;
const DEADZONE = 0.15;
let prevButtons = {};

// Button map — full emote cycling support
const BUTTON_MAP = {
    0: 'stop',           // Cross / A  — stop + cancel emotes
    1: 'play_emote',     // Circle / B — play/toggle selected emote
    2: 'play_emote',     // Square / X — play/toggle selected emote
    3: 'mode',           // Triangle / Y
    4: 'prev_emote',     // L1 — previous emote
    5: 'next_emote',     // R1 — next emote
    12: 'walk',          // D-pad Up
    13: 'backward',      // D-pad Down
    14: 'prev_emote',    // D-pad Left — previous emote
    15: 'next_emote',    // D-pad Right — next emote
};

window.addEventListener('gamepadconnected', (e) => {
    gamepadIndex = e.gamepad.index;
    gamepadStatusIndicator.classList.add('connected');
    gamepadStatusText.textContent = `🎮 ${e.gamepad.id.substring(0, 30)}`;
    log(`Gamepad connected: ${e.gamepad.id}`, 'success');
    log('Controls: Stick=Move | ✕=Stop | △=Mode | ↑↓=Fwd/Bwd | ←→/L1R1=Cycle Emotes | □○=Play Emote', 'system');
});

window.addEventListener('gamepaddisconnected', (e) => {
    if (e.gamepad.index === gamepadIndex) {
        gamepadIndex = null;
        gamepadActive = false;
        gamepadStatusIndicator.classList.remove('connected');
        gamepadStatusText.textContent = '🎮 No Gamepad';
        log('Gamepad disconnected.', 'warn');
    }
});

function applyDeadzone(value) {
    if (Math.abs(value) < DEADZONE) return 0;
    const sign = Math.sign(value);
    return sign * (Math.abs(value) - DEADZONE) / (1.0 - DEADZONE);
}

function isButtonJustPressed(gamepad, btnIndex) {
    const pressed = gamepad.buttons[btnIndex] && gamepad.buttons[btnIndex].pressed;
    const wasPrevPressed = !!prevButtons[btnIndex];
    return pressed && !wasPrevPressed;
}

function pollGamepad() {
    if (gamepadIndex === null) {
        requestAnimationFrame(pollGamepad);
        return;
    }

    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = gamepads[gamepadIndex];
    if (!gp) {
        requestAnimationFrame(pollGamepad);
        return;
    }

    for (const [btnIdx, action] of Object.entries(BUTTON_MAP)) {
        const idx = parseInt(btnIdx);
        if (isButtonJustPressed(gp, idx)) {
            if (action === 'mode') {
                setMode(!manualMode);
            } else if (action === 'stop') {
                if (trumpEmoteActive) stopTrumpEmote();
                if (saluteEmoteActive) stopSaluteEmote();
                stopEmote();
                sendCmd('stop');
                valState.textContent = 'STOPPED';
                valState.style.color = 'var(--danger)';
            } else if (action === 'play_emote') {
                playSelectedEmote();
            } else if (action === 'prev_emote') {
                selectEmote(-1);
            } else if (action === 'next_emote') {
                selectEmote(1);
            } else {
                sendCmd(action);
            }
        }
    }

    prevButtons = {};
    for (let i = 0; i < gp.buttons.length; i++) {
        prevButtons[i] = gp.buttons[i].pressed;
    }

    const rawX = gp.axes[0] || 0;
    const rawY = gp.axes[1] || 0;

    const stickX = applyDeadzone(rawX);
    const stickY = applyDeadzone(rawY);
    const stickMagnitude = Math.min(1.0, Math.sqrt(stickX * stickX + stickY * stickY));

    if (stickMagnitude > 0 && manualMode) {
        gamepadActive = true;

        // Moving the stick cancels any emote
        if (trumpEmoteActive) stopTrumpEmote();
        if (saluteEmoteActive) stopSaluteEmote();

        const forwardBackwardMagnitude = -stickY;
        currentMotion.steering = -stickX * MAX_ANGULAR_Z;
        currentMotion.speedIntensity = stickMagnitude;

        let targetDir = 'stop';
        if (forwardBackwardMagnitude > 0.1) targetDir = 'walk';
        else if (forwardBackwardMagnitude < -0.1) targetDir = 'backward';

        currentWalkDir = targetDir;
        isJoystickWalking = (targetDir !== 'stop');

        valLinearY.textContent = forwardBackwardMagnitude.toFixed(2);
        valAngularZ.textContent = currentMotion.steering.toFixed(2);
        valState.textContent = 'MOVING';
        valState.style.color = 'var(--accent)';

    } else if (gamepadActive) {
        gamepadActive = false;

        currentMotion.speedIntensity = 0;
        currentMotion.steering = 0;
        currentWalkDir = 'stop';
        isJoystickWalking = false;

        sendCmd('stop');

        valLinearY.textContent = '0.00';
        valAngularZ.textContent = '0.00';
        valState.textContent = 'IDLE';
        valState.style.color = 'var(--text-main)';
    }

    requestAnimationFrame(pollGamepad);
}

requestAnimationFrame(pollGamepad);