#!/usr/bin/env python3
"""Per-map named poses, loaded from a sidecar YAML next to the map file.

Every map needs its own surveyed poses — the dock is only at (0, 0) on maps
whose SLAM run started on the dock — so they live in git-tracked sidecar
files: maps/<name>.yaml has maps/<name>.locations.yaml beside it (see
maps/robotics_lab.locations.yaml for the format). The active map is found by
asking the running /map_server which yaml it loaded, so the poses can never
disagree with the map localization is actually using.
"""
from pathlib import Path

import rclpy
import yaml

from rcl_interfaces.srv import GetParameters
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions


def _to_degrees(direction):
    """Accept a compass name ('north', 'south_west', ...) or yaw degrees."""
    if isinstance(direction, (int, float)):
        return float(direction)
    try:
        return float(TurtleBot4Directions[str(direction).upper()])
    except KeyError:
        names = ', '.join(d.name.lower() for d in TurtleBot4Directions)
        raise ValueError(
            f"Bad direction '{direction}': use degrees or one of {names}")


def _parse_pose(pose):
    """{x:, y:, direction:} -> ([x, y], degrees) for getPoseStamped()."""
    return [float(pose['x']), float(pose['y'])], _to_degrees(pose['direction'])


def sidecar_path(map_yaml):
    p = Path(map_yaml)
    return p.with_name(p.stem + '.locations.yaml')


class MapLocations:
    """The surveyed poses for one map: dock poses + named locations."""

    def __init__(self, map_yaml, data):
        self.map_yaml = map_yaml
        try:
            dock = data['dock']
            self.undock_pose = _parse_pose(dock['undock_pose'])
            self.dock_approach_pose = _parse_pose(
                dock.get('approach_pose', dock['undock_pose']))
            self.locations = {name: _parse_pose(pose)
                              for name, pose in (data.get('locations') or {}).items()}
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f'{sidecar_path(map_yaml)}: {e!r}') from e
        self.patrol = data.get('patrol') or []
        missing = [name for name in self.patrol if name not in self.locations]
        if missing:
            raise ValueError(
                f'{sidecar_path(map_yaml)}: patrol names {missing} are not in locations')

    def pose(self, navigator, name):
        """PoseStamped for a named location."""
        position, degrees = self.locations[name]
        return navigator.getPoseStamped(position, degrees)

    def patrol_poses(self, navigator):
        """PoseStamped list for the patrol loop, in order."""
        return [self.pose(navigator, name) for name in self.patrol]


def get_active_map_yaml(node, timeout_sec=30.0):
    """Ask the running map_server which map yaml it loaded."""
    client = node.create_client(GetParameters, 'map_server/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                'map_server is not up — is the localization launch running? '
                '(SLAM sessions have no map_server and no fixed poses to load.)')
        future = client.call_async(GetParameters.Request(names=['yaml_filename']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            raise RuntimeError('map_server did not answer get_parameters in time')
        map_yaml = future.result().values[0].string_value
        if not map_yaml:
            raise RuntimeError('map_server has no yaml_filename set')
        return map_yaml
    finally:
        node.destroy_client(client)


def load_map_locations(navigator, timeout_sec=30.0):
    """Load the sidecar location file for whatever map is currently active.

    The relative map path reported by map_server resolves against the
    directory the launches were started from, so run the nodes from the
    workspace root like the launches (see docs/navigate_to_a_goal.md).
    """
    map_yaml = get_active_map_yaml(navigator, timeout_sec)
    path = sidecar_path(map_yaml)
    if not path.exists():
        raise FileNotFoundError(
            f"No location file for map '{Path(map_yaml).stem}': expected {path}. "
            'Survey the dock pose on that map and copy the format of '
            'maps/robotics_lab.locations.yaml — navigating with another map\'s '
            'poses gives AMCL a garbage initial pose.')
    return MapLocations(map_yaml, yaml.safe_load(path.read_text()))
