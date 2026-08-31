import pytest

from carrot_guide.mission import measure_command_latency

from tests.simulator import requires_simulator


@pytest.mark.integration
@requires_simulator
def test_a_velocity_command_is_acted_on_within_a_second(vehicle):
    latencies = measure_command_latency(vehicle, trials=3, step_speed_mps=3.0)
    assert len(latencies) == 3
    assert max(latencies) < 1.0
