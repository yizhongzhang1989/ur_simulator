#!/usr/bin/env python3
"""MJCF post-processor for ur_sim_config.

Consolidates the ad-hoc `sed`/inline-Python patches that `generate_mujoco_model.sh`
used to apply after `compile` converts `ur.urdf.xacro` → MJCF.

Each transform is a single function with a docstring that:
  * names the hack,
  * cites the root cause, and
  * documents how to remove it upstream.

This exists because the upstream ur_description URDF is authored for
`robot_state_publisher`/Gazebo/Isaac, not MuJoCo, and MuJoCo's URDF compiler
fuses fixed joints in ways that surface a few rough edges. A proper fix
(Round 6+ in the plan) is to hand-author an MJCF per ur_type seeded from
`mujoco_menagerie`; until then, this file is the single source of truth for
the workarounds.

Usage:
    mjcf_postprocess.py <mjcf_path> <actuators_xml> <control_mode>
"""
import os
import re
import sys


def _fix_shoulder_quat(xml: str) -> str:
    """HACK: cancel the Rz(pi) that leaks onto shoulder_link after fixed-joint fusion.

    Root cause: ur_description has a fixed joint `base_link -> base_link_inertia`
    with `rpy="0 0 pi"`. MuJoCo's URDF compiler fuses fixed joints and
    propagates the rotation to the child's <body pos/quat>. The result is a
    shoulder body at `quat="0 0 0 1"` (180° about Z), which visually freezes
    shoulder_pan_joint and also contaminates the base/shoulder mesh geoms.

    Upstream fix: remove the Rz(pi) from base_link_inertia (requires a
    ur_description change) OR hand-author the MJCF.
    """
    xml = xml.replace(
        '<body name="shoulder_link" pos="0 0 0.1625" quat="0 0 0 1">',
        '<body name="shoulder_link" pos="0 0 0.1625">',
    )
    xml = re.sub(
        r'(<geom\s+)quat="0 0 0 1"(\s+type="mesh"\s+mesh="shoulder"/>)',
        r'\1\2',
        xml,
    )
    xml = re.sub(
        r'(<geom\s+pos="0 0 0"\s+)quat="-1 0 0 0"(\s+type="mesh"\s+mesh="base"/>)',
        r'\1\2',
        xml,
    )
    return xml


def _strip_actuatorfrcrange(xml: str) -> str:
    """HACK: remove compiler-generated `actuatorfrcrange` on joints.

    Root cause: MuJoCo's URDF compiler reads URDF <limit effort="..."/> and
    emits `actuatorfrcrange` on the joint. That range then clamps the
    `<motor>` actuator independently of its `ctrlrange`, leading to surprising
    silent saturation in effort mode. We apply torque limits at the
    `<motor ctrlrange>` level (from `config/ur_types/<ur_type>.yaml`) and
    inside `gravity_compensation.py`, so the joint-level cap is redundant.

    Upstream fix: honor URDF effort limits as-is and drop our per-type YAML
    override — but then UR5e's 150 Nm would clamp UR20's 740 Nm motors.
    """
    return re.sub(r'\s*actuatorfrcrange="[^"]*"', '', xml)


def _inject_mesh_collision_flags(xml: str) -> str:
    """Duplicate each mesh <geom> as a collision-enabled companion.

    ur_description only emits visual meshes in its URDF, so MuJoCo's
    URDF compiler gives us visual-only `<geom type="mesh" .../>` entries
    with no collision companion. We:

      1. Mark the original mesh geom as visual-only
         (`group="2" contype="0" conaffinity="0"`).
      2. Emit a sibling `<geom type="mesh" .../>` for each one with
         `group="3" contype="1" conaffinity="0"`. MuJoCo's contact filter
         is `(a.contype & b.conaffinity) | (b.contype & a.conaffinity) != 0`,
         so a collision-mesh-with-conaffinity=0 cannot collide with another
         robot collision mesh (no self-collision), but WILL collide with a
         plane or external object whose default `conaffinity=1` matches the
         robot's `contype=1`.

    This trades "no self-collision" for "the robot collides with its
    environment at all", which is strictly better than the prior all-off
    behavior. Proper self-collision needs hand-authored convex decomp
    (future work, tracked in Round 6+).
    """
    mesh_geom_re = re.compile(r'<geom\b[^/]*\btype="mesh"[^/]*/>')

    def _dup(match):
        original = match.group(0)
        # Collision companion: strip the visual-only attrs so the defaults
        # for contype/conaffinity kick in (contype=1, conaffinity=0 below).
        companion = re.sub(r'\s*contype="0"', '', original)
        companion = re.sub(r'\s*conaffinity="0"', '', companion)
        companion = re.sub(r'\s*group="2"', '', companion)
        companion = companion.replace(
            'type="mesh"',
            'type="mesh" group="3" contype="1" conaffinity="0"',
        )
        return original + '\n      ' + companion

    # First ensure every mesh geom is tagged visual-only (idempotent).
    def _tag_visual(m):
        g = m.group(0)
        if 'group=' in g:
            return g
        return g.replace(
            'type="mesh"',
            'type="mesh" group="2" contype="0" conaffinity="0"',
        )

    xml = mesh_geom_re.sub(_tag_visual, xml)
    # Then duplicate each visual mesh geom with a collision companion.
    xml = re.sub(
        r'<geom\b[^/]*\btype="mesh"[^/]*group="2"[^/]*/>',
        _dup,
        xml,
    )
    return xml


def _inject_joint_friction(xml: str) -> str:
    """Add reduction-gear friction/damping to each joint (not present in URDF)."""
    base_joints = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint')
    wrist_joints = ('wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
    for j in base_joints:
        xml = xml.replace(
            f'joint name="{j}"',
            f'joint name="{j}" frictionloss="5" damping="10"',
        )
    for j in wrist_joints:
        xml = xml.replace(
            f'joint name="{j}"',
            f'joint name="{j}" frictionloss="1" damping="3"',
        )
    return xml


def _inject_actuators(xml: str, actuators_xml: str) -> str:
    return xml.replace('</mujoco>', actuators_xml + '\n</mujoco>')


def main(argv):
    if len(argv) != 4:
        print(__doc__)
        sys.exit(2)
    mjcf_path, actuator_path, control_mode = argv[1], argv[2], argv[3]
    with open(mjcf_path) as f:
        xml = f.read()
    with open(actuator_path) as f:
        actuators = f.read()

    xml = _inject_actuators(xml, actuators)
    xml = _fix_shoulder_quat(xml)
    xml = _strip_actuatorfrcrange(xml)
    xml = _inject_mesh_collision_flags(xml)
    xml = _inject_joint_friction(xml)

    with open(mjcf_path, 'w') as f:
        f.write(xml)
    print(f"  MJCF postprocessed: mode={control_mode} ({os.path.basename(mjcf_path)})")


if __name__ == "__main__":
    main(sys.argv)
