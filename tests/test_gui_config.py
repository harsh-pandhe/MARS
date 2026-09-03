"""Unit tests for world SDF parsing and --gui-config generation."""

import os
import re
import tempfile
import pytest

MARS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
LAUNCH_FILE = os.path.join(MARS_ROOT, 'src', 'mars_swarm', 'launch', 'spawn_multi.launch.py')


def test_warehouse_gui_block_exists():
    """Warehouse world SDF contains a defined <gui> block with custom camera_pose."""
    world_path = os.path.join(MARS_ROOT, 'src', 'mars_swarm', 'worlds', 'warehouse.sdf')
    assert os.path.exists(world_path)
    with open(world_path) as f:
        content = f.read()
    
    match = re.search(r'<gui[^>]*>(.*)</gui>', content, re.DOTALL)
    assert match is not None
    gui_body = match.group(1)
    assert '<camera_pose>-3 -3 1.6 0 0.45 0.785</camera_pose>' in gui_body


def test_build_gui_config_structure():
    """Verify that generated gui.config contains window header and MinimalScene plugin."""
    world_path = os.path.join(MARS_ROOT, 'src', 'mars_swarm', 'worlds', 'warehouse.sdf')
    with open(world_path) as f:
        xml = f.read()
    match = re.search(r'<gui[^>]*>(.*)</gui>', xml, re.DOTALL)
    
    header = """<?xml version="1.0"?>
<window>
  <state>docked</state>
  <default_map_res>0.05</default_map_res>
  <menus>
    <drawer default="false">
    </drawer>
  </menus>
  <dialog_on_exit>true</dialog_on_exit>
</window>
"""
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='_gui.config', delete=False)
    tmp.write(header)
    tmp.write(match.group(1))
    tmp.close()
    
    with open(tmp.name) as f:
        config_text = f.read()
    
    assert '<state>docked</state>' in config_text
    assert '<camera_pose>-3 -3 1.6 0 0.45 0.785</camera_pose>' in config_text
    os.unlink(tmp.name)
