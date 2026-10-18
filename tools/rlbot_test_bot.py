"""A minimal RLBot v5 bot for exercising the bridge.

Install the v5 Python interface in a venv:

    python3 -m venv /tmp/rlbot-venv
    /tmp/rlbot-venv/bin/pip install --pre "rlbot==2.0.0b56"

Then start the viewer with ``--rlbot`` and run this script with the matching
agent id:

    RLBOT_AGENT_ID=rlbot/opponent /tmp/rlbot-venv/bin/python tools/rlbot_test_bot.py

It drives forward and steers gently, which is enough to confirm the handshake,
game packets, ball prediction, and controller round trip.
"""

from rlbot.flat import ControllerState
from rlbot.managers import Bot


class TestBot(Bot):
    def get_output(self, packet):
        return ControllerState(throttle=1.0, boost=True, steer=0.25)


if __name__ == "__main__":
    TestBot("rlbot/opponent").run()
