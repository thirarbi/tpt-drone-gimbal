"""Regression (bug, 30 Oct): after the T3c frames.py fix the sim still integrated body Right
into the renderer's world y, mirroring one lateral axis -> positive feedback, no landing."""
from landing_sw import main


def test_sim_lands_from_an_offset_start():
    assert main.run_sim(start_xyz=(1.2, -0.8, 6.0), verbose=False)["touchdown"]
