from copy import deepcopy
from hashlib import sha256
from pathlib import Path
from tempfile import gettempdir
import xml.etree.ElementTree as ET


def _resolve_meshes(root, urdf_path, package_dir=None):
    """Resolve source mesh URIs before writing the combined URDF to /tmp."""
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if filename.startswith("package://"):
            package, relative_path = filename.removeprefix("package://").split("/", 1)
            if package_dir is None or package != package_dir.name:
                raise ValueError(f"Unsupported mesh package in {urdf_path}: {filename}")
            mesh_path = package_dir / relative_path
        else:
            mesh_path = urdf_path.parent / filename
        if not mesh_path.is_file():
            raise FileNotFoundError(f"Missing robot mesh: {mesh_path}")
        mesh.set("filename", str(mesh_path.resolve()))


def build_go2_yam_urdf():
    """Combine the bundled Unitree Go2 and I2RT YAM Pro v1 models."""
    model_dir = Path(__file__).resolve().parents[1] / "models"
    base_package = model_dir / "go2_description"
    base_urdf = base_package / "urdf" / "go2_description.urdf"
    arm_urdf = model_dir / "yam_pro" / "v1" / "yam_pro.urdf"
    arm_mjcf = arm_urdf.with_suffix(".xml")
    for source in (base_urdf, arm_urdf, arm_mjcf):
        if not source.is_file():
            raise FileNotFoundError(f"Missing bundled Go2/YAM Pro model: {source}")

    base_root = ET.parse(base_urdf).getroot()
    arm_root = ET.parse(arm_urdf).getroot()
    _resolve_meshes(base_root, base_urdf, base_package)
    _resolve_meshes(arm_root, arm_urdf)

    # The upstream URDF uses 1 Nm placeholders. Use the effort bounds from
    # I2RT's companion simulation model (10 Nm for each of the six arm joints).
    for joint in ET.parse(arm_mjcf).getroot().iter("joint"):
        lower, upper = map(float, joint.get("actuatorfrcrange").split())
        limit = arm_root.find(f"joint[@name='{joint.get('name')}']/limit")
        limit.set("effort", str(min(abs(lower), abs(upper))))

    # Both source robots call their root "base". The MPC expects the quadruped
    # frame to be "base_link"; keep the arm's root as "base".
    base_root.find("link[@name='base']").set("name", "base_link")
    for joint in base_root.findall("joint"):
        for tag in ("parent", "child"):
            link = joint.find(tag)
            if link.get("link") == "base":
                link.set("link", "base_link")

    mount_joint = ET.Element("joint", {"name": "yam_mount_joint", "type": "fixed"})
    ET.SubElement(mount_joint, "origin", {"xyz": "0.190 0 0.1027", "rpy": "0 0 0"})
    ET.SubElement(mount_joint, "parent", {"link": "base_link"})
    ET.SubElement(mount_joint, "child", {"link": "base"})

    for element in arm_root:
        if element.tag in {"link", "joint"}:
            base_root.append(deepcopy(element))
    base_root.append(mount_joint)
    base_root.set("name", "go2_yam_pro")

    # Include the composed contents and absolute mesh paths in the cache key.
    # Either source, the mount, or this checkout's location may have changed.
    contents = ET.tostring(base_root, encoding="utf-8", xml_declaration=True)
    digest = sha256(contents).hexdigest()[:16]
    output_path = Path(gettempdir()) / f"wb_mpc_go2_yam_{digest}.urdf"
    output_path.write_bytes(contents)
    return str(output_path)
