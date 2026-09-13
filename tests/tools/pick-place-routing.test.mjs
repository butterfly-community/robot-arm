import { readFile } from 'node:fs/promises';
import assert from 'node:assert/strict';
import { test } from 'node:test';

test('all pick requests pass through the scene owner; motion has no competing scene subscription', async () => {
  const flow = await readFile(new URL('../../dataflow.yml', import.meta.url), 'utf8');
  const scene = flow.split('  - id: scene-node\n')[1].split('\n  - id:')[0];
  const motion = flow.split('  - id: stararm-102-motion-node\n')[1].split('\n  - id:')[0];
  assert.match(scene, /pick_place_request: web-gateway-node\/pick_place_request/);
  assert.match(scene, /- scene_pick_place_request/);
  assert.match(motion, /pick_place_request: scene-node\/scene_pick_place_request/);
  assert.match(motion, /execution_transport: stararm-102-execution-node\/transport_state/);
  assert.doesNotMatch(motion, /source: scene-node\/world_scene|source: scene-node\/perception_state/);
});
