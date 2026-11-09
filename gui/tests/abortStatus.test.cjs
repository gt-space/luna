const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const ts = require('typescript');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../src/system/abortStatus.ts'), 'utf8');
const js = ts.transpileModule(source, { compilerOptions: { module: ts.ModuleKind.CommonJS } }).outputText;
const moduleExports = {};
vm.runInNewContext(js, { exports: moduleExports });
const { abortStatus } = moduleExports;
const status = (phase) => ({ phase, stage: 'test', request_id: 'abc', unsynchronized: false, sams: { SAM1: 'activated' } });

test('only a live Flight confirmation clears the warning', () => {
  for (const phase of ['unsent', 'preparing', 'activating', 'failed', 'partially_applied']) {
    assert.equal(abortStatus(status(phase), null, 0, true, true).confirmed, false);
    assert.equal(abortStatus(status(phase), null, 0, true, true).label, phase.replace('_', ' ').toUpperCase());
  }
  assert.equal(abortStatus(status('confirmed'), null, 0, false, true).confirmed, false);
  assert.equal(abortStatus(status('confirmed'), null, 0, true, false).confirmed, false);
  assert.equal(abortStatus(status('confirmed'), null, 0, true, true).confirmed, true);
  assert.equal(abortStatus({ ...status('confirmed'), unsynchronized: true }, null, 0, true, true).confirmed, false);
});

test('HTTP acceptance cannot confirm a pending request; failures and timeout remain visible', () => {
  const request = { id: 'def', stage: 'test', time: 100 };
  assert.equal(abortStatus(status('confirmed'), request, 200, true, true).confirmed, false);
  assert.match(abortStatus(status('confirmed'), request, 7101, true, true).label, /TIMEOUT/);
  assert.match(abortStatus(status('confirmed'), { ...request, error: 'Flight disconnected' }, 200, true, true).label, /FAILED/);
  assert.equal(abortStatus(status('partially_applied'), null, 200, true, true).label, 'PARTIALLY APPLIED');
});
