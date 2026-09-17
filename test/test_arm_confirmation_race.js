// ==============================================================================
// test_arm_confirmation_race.js — Regression Tests for Arm-Confirmation Race Fix
// ==============================================================================

const assert = require('assert');
const http = require('http');

const DEFAULT_CMD_VEL_TOKEN = 'a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890'; // gitleaks:allow
const DEFAULT_OPERATOR_TOKEN = 'test_operator_token_12345678901234567890123456789012';

process.env.PORT = '3820';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3821';
if (!process.env.ROVER_CMD_VEL_TOKEN) {
  process.env.ROVER_CMD_VEL_TOKEN = DEFAULT_CMD_VEL_TOKEN;
}
if (!process.env.ROVER_OPERATOR_TOKEN) {
  process.env.ROVER_OPERATOR_TOKEN = DEFAULT_OPERATOR_TOKEN;
}

const path = require('path');
const fs = require('fs');

const serverPath = fs.existsSync(path.join(__dirname, 'server.js'))
  ? './server.js'
  : (fs.existsSync(path.join(__dirname, '../server.js')) ? '../server.js' : './server.js');
const serverModule = require(serverPath);
const {
  app: publicApp,
  server: publicServer,
  internalCmdApp: internalApp,
  internalCmdServer: internalServer,
  autonomyState,
  injectNormalDriveStatus,
  setSerialPort,
  getNormalDriveStatusSeq
} = serverModule;

const OPERATOR_TOKEN = process.env.ROVER_OPERATOR_TOKEN || DEFAULT_OPERATOR_TOKEN;
const CMD_VEL_TOKEN = process.env.ROVER_CMD_VEL_TOKEN || DEFAULT_CMD_VEL_TOKEN;

const PUBLIC_PORT = 3820;
const INTERNAL_PORT = 3821;

function httpRequest(options, postData) {
  return new Promise((resolve, reject) => {
    const req = http.request(options, (res) => {
      let data = '';
      res.on('data', chunk => { data += chunk; });
      res.on('end', () => {
        let json = null;
        try { json = JSON.parse(data); } catch (e) {}
        resolve({ statusCode: res.statusCode, headers: res.headers, data, json });
      });
    });
    req.on('error', reject);
    if (postData !== undefined) {
      req.write(typeof postData === 'string' ? postData : JSON.stringify(postData));
    }
    req.end();
  });
}

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function runTests() {
  console.log('Running Arm Confirmation Race & Hardware Confirmation Regression Tests...\n');

  await new Promise(r => publicServer.listen(PUBLIC_PORT, '127.0.0.1', r));
  await new Promise(r => internalServer.listen(INTERNAL_PORT, '127.0.0.1', r));

  const writtenPackets = [];
  const mockSerial = {
    isOpen: true,
    write: (pkt) => {
      writtenPackets.push(Buffer.from(pkt));
    },
    close: (cb) => { if (cb) cb(); }
  };
  setSerialPort(mockSerial);

  try {
    // --------------------------------------------------------------------------
    // Test 1: Stale pre-request armed=true packet is rejected
    // --------------------------------------------------------------------------
    console.log('[Test 1] Verifying stale pre-request armed=true packet is rejected...');
    {
      // Baseline status before arm request
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        source: 0,
        cmdAge: 10,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: false
      });
      const preReqSeq = getNormalDriveStatusSeq();

      // Reset state to READY_DISARMED
      autonomyState.enabled = true;
      autonomyState.state = 'READY_DISARMED';
      serverModule.setLatestNormalDriveStatus({
        armed: false,
        mode: 0,
        seq: preReqSeq
      });

      // Start arm request
      writtenPackets.length = 0;
      let completed = false;
      const armPromise = httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': OPERATOR_TOKEN }
      }).then(res => { completed = true; return res; });

      // After 50ms, verify arm has NOT resolved using the pre-request seq
      await sleep(50);
      assert.strictEqual(completed, false, 'Arm request must not resolve from pre-request status');
      assert.strictEqual(autonomyState.state, 'READY_DISARMED', 'State must remain READY_DISARMED while waiting');

      // Now inject a genuinely NEW packet with seq > preReqSeq confirming mode 3
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        source: 0,
        cmdAge: 10,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: false
      });

      const res = await armPromise;
      assert.strictEqual(res.statusCode, 200, 'Arm request must succeed after new confirmation');
      assert.strictEqual(res.json.ok, true);
      assert.strictEqual(res.json.mode, 3);
      assert.strictEqual(autonomyState.state, 'READY_ARMED', 'State must transition to READY_ARMED only on new packet');
      console.log('✓ Test 1 Passed: Stale pre-request packet ignored; genuinely new packet required.\n');
    }

    // --------------------------------------------------------------------------
    // Test 2: Stale armed=false packet arriving after arm request, followed by new armed=true/mode-3 packet
    // --------------------------------------------------------------------------
    console.log('[Test 2] Verifying in-flight stale armed=false packet followed by armed=true/mode-3 packet...');
    {
      autonomyState.enabled = true;
      autonomyState.state = 'READY_DISARMED';
      serverModule.setLatestNormalDriveStatus({ armed: false, mode: 0, seq: getNormalDriveStatusSeq() });

      writtenPackets.length = 0;
      let completed = false;
      const armPromise = httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': OPERATOR_TOKEN }
      }).then(res => { completed = true; return res; });

      await sleep(20);
      // Verify FUNC_ARM_NORMAL_DRIVE (0x2C) was sent (cmd: [0xFF, 0xFC, len, funcId, ...])
      assert.strictEqual(writtenPackets.length, 1, 'Arm command packet 0x2C must be transmitted');
      assert.strictEqual(writtenPackets[0][3], 0x2C, 'Packet function ID must be 0x2C');

      // Simulate the exact race: in-flight stale 0x36 packet arrives with armed=false
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        source: 0,
        cmdAge: 20,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: false
      });

      await sleep(20);
      assert.strictEqual(completed, false, 'Stale armed=false packet must NOT reject or complete arm request');
      assert.strictEqual(autonomyState.state, 'READY_DISARMED', 'autonomyState must not enter READY_ARMED yet');

      // Now the ESP32 processes 0x2C and emits new packet with armed=true, mode=3
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        source: 0,
        cmdAge: 5,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: false
      });

      const res = await armPromise;
      assert.strictEqual(res.statusCode, 200, 'Arm request must complete with 200 OK');
      assert.strictEqual(res.json.ok, true);
      assert.strictEqual(res.json.status, 'ARMED');
      assert.strictEqual(res.json.mode, 3);
      assert.strictEqual(autonomyState.state, 'READY_ARMED', 'State must become READY_ARMED');
      console.log('✓ Test 2 Passed: In-flight stale armed=false tolerated without race abort; armed=true/mode-3 completes arming.\n');
    }

    // --------------------------------------------------------------------------
    // Test 3: Confirmation Timeout (500 ms)
    // --------------------------------------------------------------------------
    console.log('[Test 3] Verifying 500 ms confirmation timeout behavior...');
    {
      autonomyState.enabled = true;
      autonomyState.state = 'READY_DISARMED';
      serverModule.setLatestNormalDriveStatus({ armed: false, mode: 0, seq: getNormalDriveStatusSeq() });

      writtenPackets.length = 0;
      const t0 = Date.now();
      const timeoutRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': OPERATOR_TOKEN }
      });
      const elapsed = Date.now() - t0;

      // --------------------------------------------------------------------------
      // Test 3A: Confirmation Timeout with unconfirmed disarm (no status frame arrives)
      // --------------------------------------------------------------------------
      assert.strictEqual(timeoutRes.statusCode, 504, 'Arm timeout must return HTTP 504');
      assert.strictEqual(timeoutRes.json.ok, false);
      assert.ok(timeoutRes.json.error.includes('500ms'), 'Error message must specify 500ms timeout');
      assert.strictEqual(timeoutRes.json.disarmRequested, true, 'disarmRequested must be true');
      assert.strictEqual(timeoutRes.json.disarmConfirmed, false, 'disarmConfirmed must be false when no packet arrives');
      assert.ok(elapsed >= 780 && elapsed < 1100, `Timeout + disarm wait must take ~800ms (elapsed: ${elapsed}ms)`);
      assert.strictEqual(autonomyState.state, 'READY_DISARMED', 'State must NOT transition to READY_ARMED on timeout');

      // Verify emergency disarm 0x2D was commanded
      assert.strictEqual(writtenPackets.length, 2, 'Must transmit arm (0x2C) followed by fail-safe disarm (0x2D)');
      assert.strictEqual(writtenPackets[0][3], 0x2C, 'First packet was 0x2C');
      assert.strictEqual(writtenPackets[1][3], 0x2D, 'Second packet must be disarm 0x2D');
      console.log('✓ Test 3A Passed: 500 ms timeout enforces fail-safe disarm and reports unconfirmed when no packet arrives.\n');
    }

    // --------------------------------------------------------------------------
    // Test 3B: Timeout followed by delayed armed telemetry and subsequent confirmed disarm
    // --------------------------------------------------------------------------
    console.log('[Test 3B] Verifying timeout followed by delayed armed telemetry and subsequent confirmed disarm...');
    {
      autonomyState.enabled = true;
      autonomyState.state = 'READY_DISARMED';
      serverModule.setLatestNormalDriveStatus({ armed: false, mode: 0, seq: getNormalDriveStatusSeq() });

      writtenPackets.length = 0;
      const armPromise = httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': OPERATOR_TOKEN }
      });

      // Wait 520ms for arm timeout to fire and disarm to be transmitted
      await sleep(520);
      assert.strictEqual(writtenPackets.length, 2, 'Arm (0x2C) and Disarm (0x2D) must be transmitted');
      assert.strictEqual(writtenPackets[1][3], 0x2D, 'Second packet must be 0x2D');

      // 1. Simulate delayed armed telemetry arriving after arm timeout
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        source: 0,
        cmdAge: 10,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: false
      });

      // Verify motion remains blocked despite delayed armed frame!
      assert.notStrictEqual(autonomyState.state, 'READY_ARMED', 'State must NEVER become READY_ARMED from delayed packet');
      assert.strictEqual(autonomyState.state, 'READY_DISARMED');

      // Test cmd_vel is rejected
      const blockedCmdRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': CMD_VEL_TOKEN }
      }, { linear: { x: 0.1 }, angular: { z: 0.0 } });
      assert.strictEqual(blockedCmdRes.statusCode, 403, 'cmd_vel must be rejected while arm unconfirmed/disarming');

      // 2. Now simulate subsequent disarm status frame arriving from hardware
      await sleep(30);
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        source: 0,
        cmdAge: 10,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: false
      });

      const res = await armPromise;
      assert.strictEqual(res.statusCode, 504, 'Arm request must return HTTP 504');
      assert.strictEqual(res.json.ok, false);
      assert.strictEqual(res.json.disarmRequested, true, 'disarmRequested must be true');
      assert.strictEqual(res.json.disarmConfirmed, true, 'disarmConfirmed must be true from subsequent status frame');
      assert.strictEqual(serverModule.getLatestNormalDriveStatus().armed, false, 'Hardware telemetry confirms armed=false');
      assert.strictEqual(autonomyState.state, 'READY_DISARMED', 'Motion remains safely blocked');

      console.log('✓ Test 3B Passed: Timeout followed by delayed armed telemetry handled safely; subsequent disarm confirmed.\n');
    }

    // --------------------------------------------------------------------------
    // Test 4: Mode Validation (armed=true but mode != 3 does not complete arming)
    // --------------------------------------------------------------------------
    console.log('[Test 4] Verifying mode != 3 does not satisfy arm confirmation...');
    {
      autonomyState.enabled = true;
      autonomyState.state = 'READY_DISARMED';
      serverModule.setLatestNormalDriveStatus({ armed: false, mode: 0, seq: getNormalDriveStatusSeq() });

      let completed = false;
      const armPromise = httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': OPERATOR_TOKEN }
      }).then(res => { completed = true; return res; });

      await sleep(20);
      // Inject packet with armed=true but mode=0 (LOCKED)
      injectNormalDriveStatus({
        armed: true,
        mode: 0,
        source: 0,
        cmdAge: 10,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: true
      });

      await sleep(20);
      assert.strictEqual(completed, false, 'Mode 0 must NOT complete arming even if armed=true');

      // Now inject valid mode 3
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        source: 0,
        cmdAge: 10,
        reqLinear: 0,
        reqAngular: 0,
        limLinear: 0,
        limAngular: 0,
        lockStatus: false
      });

      const res = await armPromise;
      assert.strictEqual(res.statusCode, 200);
      assert.strictEqual(res.json.mode, 3);
      console.log('✓ Test 4 Passed: Both armed=true AND mode=3 strictly required.\n');
    }

    // --------------------------------------------------------------------------
    // Test 5: No cmd_vel before confirmed READY_ARMED
    // --------------------------------------------------------------------------
    console.log('[Test 5] Verifying /api/cmd_vel rejects when not confirmed READY_ARMED...');
    {
      // Case A: State is READY_DISARMED -> Rejected
      autonomyState.enabled = true;
      autonomyState.state = 'READY_DISARMED';
      serverModule.setLatestNormalDriveStatus({ armed: false, mode: 0 });

      const disarmedRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': CMD_VEL_TOKEN }
      }, { linear: { x: 0.1 }, angular: { z: 0.0 } });
      assert.strictEqual(disarmedRes.statusCode, 403);
      assert.strictEqual(disarmedRes.json.error, 'Rover is disarmed');

      // Case B: State is READY_ARMED but hardware reported mode != 3 -> Rejected
      autonomyState.state = 'READY_ARMED';
      serverModule.setLatestNormalDriveStatus({ armed: true, mode: 0 }); // unconfirmed mode
      const unconfirmedModeRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': CMD_VEL_TOKEN }
      }, { linear: { x: 0.1 }, angular: { z: 0.0 } });
      assert.strictEqual(unconfirmedModeRes.statusCode, 403);
      assert.strictEqual(unconfirmedModeRes.json.error, 'Rover is disarmed');

      // Case C: Fully confirmed (READY_ARMED, armed=true, mode=3) -> Accepted
      serverModule.setLatestNormalDriveStatus({ armed: true, mode: 3 });
      const acceptedRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': CMD_VEL_TOKEN }
      }, { linear: { x: 0.0 }, angular: { z: 0.5 } });
      assert.strictEqual(acceptedRes.statusCode, 200);
      assert.strictEqual(acceptedRes.json.ok, true);
      assert.strictEqual(acceptedRes.json.state, 'ACTIVE');
      console.log('✓ Test 5 Passed: /api/cmd_vel strictly fail-closed until hardware-confirmed READY_ARMED (mode 3).\n');
    }

    console.log('All 5 Arm Confirmation Race & Hardware Confirmation Regression Tests PASSED!\n');

  } finally {
    await new Promise(r => publicServer.close(r));
    await new Promise(r => internalServer.close(r));
  }
}

if (require.main === module) {
  runTests()
    .then(() => {
      process.exit(0);
    })
    .catch(err => {
      console.error('\n[TEST FAILURE]:', err);
      process.exit(1);
    });
}

module.exports = { runTests };
