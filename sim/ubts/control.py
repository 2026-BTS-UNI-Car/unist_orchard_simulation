"""Simulation-only grace for physical deceleration in a slow rendered world."""
from scripts.rover_control_core import RoverControl


class SimulationControl(RoverControl):
    RELEASE_GRACE_S = 10.0

    def dispatch(self, request):
        result = super().dispatch(request)
        if request.get('op') == 'release':
            # The target is already zero. Physics ramps in simulation seconds;
            # its completion can exceed the real ESP's 1.1 wall-second grace.
            # Held drive still expires after 0.75s; pause/STOP still hard-stop.
            self.input_until = self.clock() + self.RELEASE_GRACE_S
        return result
