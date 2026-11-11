import json
import time
from pathlib import Path

state = json.loads(Path('/runtime/simulation_status.json').read_text())
assert time.time() - state['updated_at'] < 5, 'simulation adapter stale'
assert all(state['sensors_alive'].values()), state['sensors_alive']
