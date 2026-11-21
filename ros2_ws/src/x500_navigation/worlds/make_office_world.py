#!/usr/bin/env python3
"""Generate office.sdf: an indoor office floor for the x500 + 2D lidar sim.

    python3 make_office_world.py            # writes office.sdf next to this file

Layout (metres, x east, y north; floor is x -12..12, y -8..8, walls 3 m high,
no ceiling so the Gazebo camera can look in from above):

    y=8  +-----------------+----------+----------+
         |  open-plan      | meeting  | kitchen  |
         |  desks, pillars | room     |          |
    1.25 +---D-------D-----+---D------+---D------+
         |                 corridor                  (2.5 m wide)
   -1.25 +-------D-----D---+---D--+--- lobby opening ---+
         |  office A | office B | server |  lobby      G  <- entrance
    y=-8 +-----------+----------+--------+-------------+
       x=-12        -6          0        4            12

D = 1.6 m doorway. G = closed glass entrance door (the floor is sealed). Height rule for furniture, so the 2D lidar (which only sees
the plane it scans, ~0.3 m above the drone) and the drone agree on what is an
obstacle when flying at the default 1.5 m:
  * low furniture (desks, tables, chairs, sofas, counters) is at most 1.1 m
    tall: the drone flies over it, the lidar does not see it;
  * tall furniture (cabinets, shelves, racks, fridge, plants, pillars) is at
    least 2.0 m tall: the lidar sees it, Nav2 plans around it.
Nothing is between 1.1 m and 2.0 m, where the drone could hit something the
lidar cannot see.
"""

import math
import os

WALL_H = 3.0
INNER_T = 0.15   # interior wall thickness
OUTER_T = 0.25   # exterior wall thickness
DOOR_W = 1.6   # doorway width: x500 is ~0.76 m across incl. propellers

COLORS = {
    'wall': (0.92, 0.90, 0.85),
    'outer': (0.80, 0.78, 0.74),
    'floor': (0.45, 0.50, 0.58),      # blue-grey carpet
    'corridor': (0.70, 0.68, 0.64),   # stone tiles
    'lobby': (0.76, 0.72, 0.64),
    'desk': (0.72, 0.56, 0.38),
    'chair': (0.15, 0.15, 0.17),
    'cabinet': (0.55, 0.57, 0.60),
    'shelf': (0.50, 0.36, 0.24),
    'rack': (0.08, 0.08, 0.10),
    'pillar': (0.85, 0.85, 0.85),
    'plant_pot': (0.40, 0.30, 0.22),
    'plant': (0.18, 0.45, 0.20),
    'sofa': (0.25, 0.35, 0.55),
    'counter': (0.85, 0.85, 0.82),
    'fridge': (0.90, 0.90, 0.92),
    'white': (0.97, 0.97, 0.97),
    'reception': (0.30, 0.30, 0.34),
    'glass': (0.55, 0.75, 0.85),
}

models = []


def _material(color):
    r, g, b = COLORS[color]
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def box(name, cx, cy, sx, sy, sz, color, z0=0.0, yaw=0.0, collide=True):
    """Static box with its bottom at z0."""
    geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    col = f'<collision name="c">{geom}</collision>' if collide else ''
    models.append(
        f'<model name="{name}"><static>true</static>'
        f'<pose>{cx:.3f} {cy:.3f} {z0 + sz / 2:.3f} 0 0 {yaw:.4f}</pose>'
        f'<link name="link">{col}<visual name="v">{geom}{_material(color)}</visual>'
        f'</link></model>')


def cylinder(name, cx, cy, r, h, color, z0=0.0):
    geom = f'<geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>'
    models.append(
        f'<model name="{name}"><static>true</static>'
        f'<pose>{cx:.3f} {cy:.3f} {z0 + h / 2:.3f} 0 0 0</pose>'
        f'<link name="link"><collision name="c">{geom}</collision>'
        f'<visual name="v">{geom}{_material(color)}</visual></link></model>')


_wall_count = [0]


def wall(x1, y1, x2, y2, doors=(), t=INNER_T, color='wall'):
    """Axis-aligned wall from (x1,y1) to (x2,y2) with doorway centres `doors`."""
    horizontal = abs(y2 - y1) < 1e-9
    a0, a1 = (min(x1, x2), max(x1, x2)) if horizontal else (min(y1, y2), max(y1, y2))
    cuts = sorted((d - DOOR_W / 2, d + DOOR_W / 2) for d in doors)
    pos, segments = a0, []
    for c0, c1 in cuts:
        if c0 > pos:
            segments.append((pos, c0))
        pos = max(pos, c1)
    if pos < a1:
        segments.append((pos, a1))
    for s0, s1 in segments:
        _wall_count[0] += 1
        n = f'wall_{_wall_count[0]:02d}'
        mid, length = (s0 + s1) / 2, s1 - s0
        if horizontal:
            box(n, mid, y1, length, t, WALL_H, color)
        else:
            box(n, x1, mid, t, length, WALL_H, color)


def desk_with_chair(name, cx, cy, facing):
    """1.6 x 0.8 desk (0.75 m) with a chair (0.95 m) on the side it faces."""
    box(f'{name}_desk', cx, cy, 1.6, 0.8, 0.75, 'desk')
    box(f'{name}_monitor', cx, cy + 0.2 * (-facing), 0.6, 0.05, 0.35, 'chair', z0=0.75)
    box(f'{name}_chair', cx, cy + 0.75 * facing, 0.5, 0.5, 0.95, 'chair')


def plant(name, cx, cy):
    """Tall indoor plant: pot + foliage column, 2.0 m in total."""
    cylinder(f'{name}_pot', cx, cy, 0.25, 0.5, 'plant_pot')
    cylinder(f'{name}_leaves', cx, cy, 0.35, 1.5, 'plant', z0=0.5)


def build():
    # Floors (thin slabs on the ground plane, different colour per zone).
    box('floor_rooms_north', 0, 4.625, 24, 6.75, 0.02, 'floor', collide=False)
    box('floor_rooms_south', -4, -4.625, 16, 6.75, 0.02, 'floor', collide=False)
    box('floor_lobby', 8, -4.625, 8, 6.75, 0.02, 'lobby', collide=False)
    box('floor_corridor', 0, 0, 24, 2.5, 0.02, 'corridor', collide=False)

    # Exterior walls; lobby entrance on the east side, with a closed glass door
    # (2.1 m) and wall above it. Closed so the floor is sealed: explore_lite
    # would otherwise see the outdoors as unexplored space and fly out.
    wall(-12, 8, 12, 8, t=OUTER_T, color='outer')
    wall(-12, -8, 12, -8, t=OUTER_T, color='outer')
    wall(-12, -8, -12, 8, t=OUTER_T, color='outer')
    wall(12, -8, 12, 8, doors=[-5.0], t=OUTER_T, color='outer')
    box('lb_entrance_door', 12.0, -5.0, 0.06, DOOR_W, 2.1, 'glass')
    box('lb_entrance_header', 12.0, -5.0, OUTER_T, DOOR_W, WALL_H - 2.1, 'outer', z0=2.1)

    # Corridor walls with doorways (lobby side is a wide 4 m opening).
    wall(-12, 1.25, 12, 1.25, doors=[-9.0, -3.0, 1.5, 7.5])
    wall(-12, -1.25, 4, -1.25, doors=[-7.5, -1.5, 2.0])
    wall(4, -1.25, 6, -1.25)
    wall(10, -1.25, 12, -1.25)

    # Room dividers.
    wall(0, 1.25, 0, 8)       # open-plan | meeting
    wall(6, 1.25, 6, 8)       # meeting | kitchen
    wall(-6, -8, -6, -1.25)   # office A | office B
    wall(0, -8, 0, -1.25)     # office B | server
    wall(4, -8, 4, -1.25)     # server | lobby

    # Open-plan office: three pods of four desks, two pillars, shelves, plants.
    for i, x in enumerate((-10.0, -6.8, -3.6)):
        desk_with_chair(f'op{i}a', x, 3.2, -1)
        desk_with_chair(f'op{i}b', x, 4.0, +1)
        desk_with_chair(f'op{i}c', x, 6.2, -1)
        desk_with_chair(f'op{i}d', x, 7.0, +1)
    box('op_pillar_1', -8.4, 5.1, 0.45, 0.45, WALL_H, 'pillar')
    box('op_pillar_2', -5.2, 5.1, 0.45, 0.45, WALL_H, 'pillar')
    box('op_shelf_west', -11.65, 4.6, 0.4, 3.0, 2.1, 'shelf')
    box('op_cabinet_1', -1.6, 7.6, 1.2, 0.5, 2.0, 'cabinet')
    plant('op_plant_1', -0.6, 2.0)
    plant('op_plant_2', -11.4, 7.4)

    # Meeting room: long table, chairs, whiteboard, cabinet, plant.
    box('mt_table', 3.0, 4.8, 3.0, 1.2, 0.75, 'desk')
    for k, x in enumerate((1.9, 3.0, 4.1)):
        box(f'mt_chair_n{k}', x, 5.75, 0.5, 0.5, 0.95, 'chair')
        box(f'mt_chair_s{k}', x, 3.85, 0.5, 0.5, 0.95, 'chair')
    box('mt_whiteboard', 5.88, 4.8, 0.06, 2.4, 1.2, 'white', z0=0.9)
    box('mt_cabinet', 0.5, 7.55, 0.8, 0.6, 2.0, 'cabinet')
    plant('mt_plant', 5.4, 7.4)

    # Kitchen: counter, fridge, vending machine, round table + chairs.
    box('kt_counter', 9.0, 7.55, 4.4, 0.65, 0.9, 'counter')
    box('kt_fridge', 11.5, 7.45, 0.75, 0.75, 2.0, 'fridge')
    box('kt_vending', 11.5, 2.0, 0.9, 0.8, 2.0, 'reception')
    cylinder('kt_table', 8.8, 4.5, 0.6, 0.75, 'white')
    for k in range(4):
        a = k * math.pi / 2
        box(f'kt_chair_{k}', 8.8 + 0.95 * math.cos(a), 4.5 + 0.95 * math.sin(a),
            0.45, 0.45, 0.9, 'chair')
    box('kt_cupboard', 6.5, 7.6, 0.8, 0.5, 2.1, 'cabinet')

    # Office A and B: desk, chair, filing cabinet, bookshelf / plant.
    desk_with_chair('oa', -9.5, -6.4, +1)
    box('oa_cabinet', -11.6, -3.0, 0.5, 1.2, 2.0, 'cabinet')
    box('oa_shelf', -7.0, -7.65, 1.6, 0.4, 2.1, 'shelf')
    desk_with_chair('ob', -3.5, -6.4, +1)
    box('ob_cabinet', -5.6, -3.0, 0.5, 1.2, 2.0, 'cabinet')
    plant('ob_plant', -0.6, -7.4)

    # Server room: two rows of racks against the side walls.
    for k, y in enumerate((-7.3, -6.3, -5.3, -4.3)):
        box(f'sv_rack_w{k}', 0.45, y, 0.6, 0.95, 2.1, 'rack')
        box(f'sv_rack_e{k}', 3.55, y, 0.6, 0.95, 2.1, 'rack')

    # Lobby: reception desk, sofas, plants.
    box('lb_reception', 8.0, -7.0, 2.6, 0.7, 1.1, 'reception')
    box('lb_sofa_1', 11.4, -3.2, 0.8, 2.0, 0.8, 'sofa')
    box('lb_sofa_2', 4.6, -5.0, 0.8, 2.0, 0.8, 'sofa')
    plant('lb_plant_1', 4.6, -7.4)
    plant('lb_plant_2', 11.4, -7.4)
    plant('lb_plant_3', 11.4, -1.8)


def world_sdf():
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by make_office_world.py; edit that script, not this file. -->
<sdf version="1.9">
  <world name="office">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.80 0.85 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 50 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.3 0.4 -0.87</direction>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    <light name="fill" type="directional">
      <pose>0 0 50 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>0.4</intensity>
      <direction>0.4 -0.3 -0.86</direction>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0 0 0 1</specular>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
    {chr(10).join('    ' + m for m in models).strip()}
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
'''


if __name__ == '__main__':
    build()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'office.sdf')
    with open(out, 'w') as f:
        f.write(world_sdf())
    print(f'wrote {out} ({len(models)} models)')
