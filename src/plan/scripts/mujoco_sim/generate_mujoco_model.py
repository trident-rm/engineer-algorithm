#!/usr/bin/env python3
"""Generate the versioned MuJoCo model from the robot URDF and STL assets."""

import argparse
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
DEFAULT_URDF = (
    SCRIPT_DIRECTORY.parents[2]
    / "arms"
    / "combine_urdf"
    / "urdf"
    / "combine_urdf.urdf"
)
ASSET_DIRECTORY = SCRIPT_DIRECTORY / "mujoco_assets"
COLLISION_DIRECTORY = ASSET_DIRECTORY / "meshes" / "collision"
VISUAL_DIRECTORY = ASSET_DIRECTORY / "meshes" / "visual"
MODEL_PATH = ASSET_DIRECTORY / "robot_scene.xml"
DEFAULT_CORE_CENTER = np.array([0.45, 0.0, 0.4])
DEFAULT_CORE_SIZE = np.array([0.14, 0.06, 0.06])
MAX_VISUAL_FACES = 180_000


def _mesh_path(filename, urdf_path):
    if filename.startswith("package://"):
        package_path = filename.removeprefix("package://")
        package, relative_path = package_path.split("/", 1)
        package_root = urdf_path.parent.parent
        if package_root.name != package:
            raise ValueError(f"Cannot resolve package mesh path: {filename}")
        return package_root / relative_path
    path = Path(filename)
    return path if path.is_absolute() else urdf_path.parent / path


def _origin_transform(parent, tag):
    element = parent.find(tag)
    if element is None:
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    position = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    rpy = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    quaternion = Rotation.from_euler("xyz", rpy).as_quat()[[3, 0, 1, 2]]
    return position, quaternion


def _build_collision_urdf(urdf_path, temporary_directory):
    root = ET.parse(urdf_path).getroot()
    hull_directory = Path(temporary_directory) / "hulls"
    hull_directory.mkdir()
    for link in root.findall("link"):
        collisions = link.findall("collision")
        mesh_collision = next(
            (
                collision
                for collision in collisions
                if collision.find("./geometry/mesh") is not None
            ),
            None,
        )
        if mesh_collision is None:
            continue
        source_mesh = mesh_collision.find("./geometry/mesh")
        source_path = _mesh_path(source_mesh.get("filename"), urdf_path)
        if not source_path.is_file():
            raise FileNotFoundError(f"URDF collision mesh does not exist: {source_path}")
        mesh = trimesh.load_mesh(source_path, process=False)
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            raise ValueError(f"Expected a triangle mesh with faces: {source_path}")
        hull = mesh.convex_hull
        if len(hull.faces) == 0:
            raise ValueError(f"Could not construct a collision hull for {source_path}")
        hull_name = f"{link.get('name')}.stl"
        hull.export(hull_directory / hull_name, file_type="stl")
        for collision in collisions:
            link.remove(collision)
        collision = ET.SubElement(link, "collision", name=f"{link.get('name')}_hull")
        origin = mesh_collision.find("origin")
        if origin is not None:
            collision.append(ET.fromstring(ET.tostring(origin)))
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "mesh", filename=hull_name)

    mujoco_extension = root.find("mujoco")
    if mujoco_extension is None:
        mujoco_extension = ET.SubElement(root, "mujoco")
    compiler = mujoco_extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_extension, "compiler")
    compiler.set("meshdir", str(hull_directory))

    root_link = {
        link.get("name") for link in root.findall("link")
    } - {
        child.get("link")
        for joint in root.findall("joint")
        if (child := joint.find("child")) is not None
    }
    if len(root_link) != 1:
        raise ValueError(f"Expected one URDF root link, found {sorted(root_link)}")
    core_link = ET.SubElement(root, "link", name="simulation_core")
    collision = ET.SubElement(core_link, "collision", name="simulation_core_box")
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        geometry,
        "box",
        size=" ".join(f"{value:.9g}" for value in DEFAULT_CORE_SIZE),
    )
    core_joint = ET.SubElement(root, "joint", name="simulation_core_fixed", type="fixed")
    ET.SubElement(
        core_joint,
        "origin",
        xyz=" ".join(f"{value:.9g}" for value in DEFAULT_CORE_CENTER),
    )
    ET.SubElement(core_joint, "parent", link=next(iter(root_link)))
    ET.SubElement(core_joint, "child", link="simulation_core")
    urdf_output = Path(temporary_directory) / "robot_collision.urdf"
    ET.ElementTree(root).write(urdf_output, encoding="utf-8", xml_declaration=True)
    return urdf_output


def _add_visual_geometries(root, urdf_path):
    urdf = ET.parse(urdf_path).getroot()
    worldbody = root.find("worldbody")
    asset = root.find("asset")
    for link in urdf.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh = visual.find("./geometry/mesh")
        if mesh is None or not mesh.get("filename"):
            continue
        source_path = _mesh_path(mesh.get("filename"), urdf_path).resolve()
        if not source_path.is_file():
            raise FileNotFoundError(f"URDF visual mesh does not exist: {source_path}")
        source_mesh = trimesh.load_mesh(source_path, process=False)
        if not isinstance(source_mesh, trimesh.Trimesh) or len(source_mesh.faces) == 0:
            raise ValueError(f"Expected a triangle visual mesh: {source_path}")
        if len(source_mesh.faces) > MAX_VISUAL_FACES:
            source_mesh = source_mesh.simplify_quadric_decimation(
                face_count=MAX_VISUAL_FACES
            )
        visual_name = f"{link.get('name')}_visual"
        visual_path = VISUAL_DIRECTORY / f"{visual_name}.stl"
        source_mesh.export(visual_path, file_type="stl")
        ET.SubElement(
            asset,
            "mesh",
            name=visual_name,
            file=f"meshes/visual/{visual_path.name}",
        )

        color = visual.find("material/color")
        rgba = color.get("rgba", "0.8 0.8 0.8 1") if color is not None else "0.8 0.8 0.8 1"
        body = worldbody.find(f".//body[@name='{link.get('name')}']")
        if body is None:
            collision_geom = worldbody.find(f"geom[@name='{link.get('name')}_hull']")
            if collision_geom is None:
                raise ValueError(f"Missing imported geometry for fixed link {link.get('name')}")
            visual_geom = ET.SubElement(worldbody, "geom")
            for attribute in ("pos", "quat"):
                if collision_geom.get(attribute):
                    visual_geom.set(attribute, collision_geom.get(attribute))
        else:
            visual_geom = ET.SubElement(body, "geom")
            position, quaternion = _origin_transform(visual, "origin")
            visual_geom.set("pos", " ".join(f"{value:.12g}" for value in position))
            visual_geom.set("quat", " ".join(f"{value:.12g}" for value in quaternion))
        visual_geom.set("name", f"{link.get('name')}_visual")
        visual_geom.set("type", "mesh")
        visual_geom.set("mesh", visual_name)
        visual_geom.set("rgba", rgba)
        visual_geom.set("contype", "0")
        visual_geom.set("conaffinity", "0")
        visual_geom.set("group", "2")
        visual_geom.set("mass", "0")

    for joint in urdf.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        child = joint.find("child")
        parent = joint.find("parent")
        if child is None or parent is None:
            continue
        child_name = child.get("link")
        parent_name = parent.get("link")
        body = worldbody.find(f".//body[@name='{parent_name}']")
        if body is None:
            continue
        position, quaternion = _origin_transform(joint, "origin")
        site = ET.SubElement(body, "site", name=f"tcp_site_{child_name}", size="0.004")
        site.set("pos", " ".join(f"{value:.12g}" for value in position))
        site.set("quat", " ".join(f"{value:.12g}" for value in quaternion))


def generate(urdf_path):
    ASSET_DIRECTORY.mkdir(parents=True, exist_ok=True)
    COLLISION_DIRECTORY.mkdir(parents=True, exist_ok=True)
    VISUAL_DIRECTORY.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="build_type3_mjcf_") as temporary_directory:
        collision_urdf = _build_collision_urdf(urdf_path, temporary_directory)
        imported_model = mujoco.MjModel.from_xml_path(str(collision_urdf))
        imported_path = Path(temporary_directory) / "imported.xml"
        mujoco.mj_saveLastXML(str(imported_path), imported_model)
        root = ET.parse(imported_path).getroot()
        compiler = root.find("compiler")
        if compiler is not None:
            compiler.attrib.pop("meshdir", None)
        for mesh in root.findall("./asset/mesh"):
            filename = Path(mesh.get("file")).name
            shutil.copy2(
                Path(temporary_directory) / "hulls" / filename,
                COLLISION_DIRECTORY / filename,
            )
            mesh.set("file", f"meshes/collision/{filename}")

        _add_visual_geometries(root, urdf_path)
        worldbody = root.find("worldbody")
        for geom in root.findall(".//geom"):
            name = geom.get("name", "")
            if name.endswith("_hull"):
                geom.set("contype", "1")
                geom.set("conaffinity", "0")
        ET.SubElement(
            worldbody,
            "geom",
            name="simulation_ground",
            type="plane",
            size="2 2 0.1",
            pos="0 0 -0.25",
            friction="0.8 0.02 0.002",
            rgba="0.35 0.35 0.35 1",
            contype="0",
            conaffinity="1",
        )
        option = root.find("option")
        if option is None:
            option = ET.SubElement(root, "option")
        option.set("timestep", "0.002")
        option.set("gravity", "0 0 -9.81")
        option.set("integrator", "implicitfast")
        actuator = root.find("actuator")
        if actuator is None:
            actuator = ET.SubElement(root, "actuator")
        urdf = ET.parse(urdf_path).getroot()
        for joint in urdf.findall("joint"):
            if joint.get("type") != "revolute":
                continue
            limit = joint.find("limit")
            ET.SubElement(
                actuator,
                "position",
                name=f"{joint.get('name')}_position",
                joint=joint.get("name"),
                kp="100",
                kv="20",
                ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
                ctrllimited="true",
                forcerange="-40 40",
                forcelimited="true",
            )
        root.set("model", "combine_urdf_type3")
        ET.indent(root, space="  ")
        ET.ElementTree(root).write(MODEL_PATH, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    print(
        f"Generated {MODEL_PATH}: {model.nbody} bodies, {model.ngeom} geoms, "
        f"{model.nmesh} meshes, {model.nu} actuators."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    args = parser.parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        parser.error(f"URDF file does not exist: {urdf_path}")
    generate(urdf_path)


if __name__ == "__main__":
    main()
