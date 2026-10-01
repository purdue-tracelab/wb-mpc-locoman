from copy import deepcopy
from pathlib import Path
from tempfile import gettempdir
import xml.etree.ElementTree as ET


def build_go2_yam_urdf():
    repo_dir = Path(__file__).resolve().parents[1]
    workspace_dir = repo_dir.parent
    base_urdf = workspace_dir / "unitree_ros" / "robots" / "b2_description" / "urdf" / "b2_description.urdf"
    base_mesh_dir = base_urdf.parent.parent / "meshes"
    arm_urdf = workspace_dir / "i2rt" / "i2rt" / "robot_models" / "arm" / "yam_pro" / "v1" / "yam_pro.urdf"
    arm_mesh_dir = arm_urdf.parent / "assets"

    output_path = Path(gettempdir()) / "wb_mpc_go2_yam.urdf"
    if output_path.exists() and output_path.stat().st_mtime >= min(base_urdf.stat().st_mtime, arm_urdf.stat().st_mtime):
        return str(output_path)

    base_root = ET.parse(base_urdf).getroot()
    arm_root = ET.parse(arm_urdf).getroot()

    for mesh in base_root.iter("mesh"):
        mesh.set("filename", str(base_mesh_dir / Path(mesh.get("filename")).name))
    for mesh in arm_root.iter("mesh"):
        mesh.set("filename", str(arm_mesh_dir / Path(mesh.get("filename")).name))

    mount_joint = ET.Element("joint", {"name": "yam_mount_joint", "type": "fixed"})
    ET.SubElement(mount_joint, "origin", {"xyz": "0.190 0 0.1027", "rpy": "0 0 0"})
    ET.SubElement(mount_joint, "parent", {"link": "base_link"})
    ET.SubElement(mount_joint, "child", {"link": "base"})

    for element in arm_root:
        if element.tag in {"link", "joint"}:
            base_root.append(deepcopy(element))
    base_root.append(mount_joint)
    base_root.set("name", "go2_yam_pro")

    ET.ElementTree(base_root).write(output_path, encoding="utf-8", xml_declaration=True)
    return str(output_path)