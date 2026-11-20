"""ค่าที่ขึ้นกับสนามแต่ละอัน: ชื่อโลกใน Gazebo, จุดวางหุ่น และระยะจับทางแยก."""

import os
import re
import xml.etree.ElementTree as ET

DEFAULT_WORLD = 'Map1'
WORLD_NAME_RE = re.compile(r'^[A-Za-z0-9_]+$')

# name, x (m), y (m), yaw (rad) · หุ่นตัวแรกอยู่หน้าสุด หันเข้าเขาวงกต (+x)
_SEED7_SPAWNS = (
    ('mbot1', -1.95, -1.25, 0.0),
    ('mbot2', -2.20, -1.25, 0.0),
    ('mbot3', -2.45, -1.25, 0.0),
)
# Map1: กล่องเริ่มกว้าง 0.80 ม. กลางอยู่ที่ (-3.5, 3) เส้นนำเข้าวิ่งไป +x หุ่นเรียงแถวบนเส้นนั้น
_MAP1_SPAWNS = (
    ('mbot1', -3.25, 3.00, 0.0),
    ('mbot2', -3.50, 3.00, 0.0),
    ('mbot3', -3.75, 3.00, 0.0),
)
# Map1_Walled/Map2: แผ่นเริ่มที่ (-3, 3) เส้นแรกวิ่งไป +x ตัวที่ 2–3 ต่อแถวด้านหลังแผ่นเริ่ม
_GRID16_SPAWNS = (
    ('mbot1', -3.00, 3.00, 0.0),
    ('mbot2', -3.25, 3.00, 0.0),
    ('mbot3', -3.50, 3.00, 0.0),
)

PROFILES = {
    'mbot2_seed7_maze': {'spawns': _SEED7_SPAWNS, 'junction_match_radius': 0.25},
    'Map1': {'spawns': _MAP1_SPAWNS, 'junction_match_radius': 0.2},
    'Map1_Walled': {'spawns': _GRID16_SPAWNS, 'junction_match_radius': 0.2},
    'Map2': {'spawns': _GRID16_SPAWNS, 'junction_match_radius': 0.2},
}


def gazebo_world_name(sdf_path):
    """อ่านชื่อจาก <world name="..."> ซึ่งอาจไม่ตรงกับชื่อไฟล์."""
    world = ET.parse(sdf_path).getroot().find('world')
    if world is None or not world.get('name'):
        raise ValueError(f'{os.path.basename(sdf_path)} has no <world name>')
    return world.get('name')


def resolve(worlds_dir, world):
    """คืนค่าทั้งหมดที่ launch ต้องใช้สำหรับสนาม `world`."""
    if not isinstance(world, str) or not WORLD_NAME_RE.match(world):
        raise ValueError(f'world must match {WORLD_NAME_RE.pattern}: {world!r}')
    sdf_path = os.path.join(worlds_dir, f'{world}.sdf')
    if not os.path.isfile(sdf_path):
        raise ValueError(f'world file not found: {sdf_path}')
    profile = PROFILES.get(world, PROFILES[DEFAULT_WORLD])
    return {
        'file': sdf_path,
        'gz_name': gazebo_world_name(sdf_path),
        'spawns': profile['spawns'],
        'junction_match_radius': profile['junction_match_radius'],
    }
