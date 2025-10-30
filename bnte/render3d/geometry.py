"""Primitive geometry utilities for BNTE 3D stage generation."""
from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, sin, sqrt
from typing import Iterable, List, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Triangle = Tuple[int, int, int]


@dataclass
class Quaternion:
    """Simple quaternion helper."""

    w: float
    x: float
    y: float
    z: float

    @staticmethod
    def identity() -> "Quaternion":
        return Quaternion(1.0, 0.0, 0.0, 0.0)

    @staticmethod
    def from_axis_angle(axis: Vector3, angle_rad: float) -> "Quaternion":
        ax, ay, az = axis
        mag = sqrt(ax * ax + ay * ay + az * az)
        if mag == 0.0:
            return Quaternion.identity()
        ax /= mag
        ay /= mag
        az /= mag
        half = angle_rad * 0.5
        s = sin(half)
        return Quaternion(cos(half), ax * s, ay * s, az * s)

    def as_rotation_matrix(self) -> List[List[float]]:
        w, x, y, z = self.w, self.x, self.y, self.z
        ww, xx, yy, zz = w * w, x * x, y * y, z * z
        wx, wy, wz = w * x, w * y, w * z
        xy, xz, yz = x * y, x * z, y * z
        return [
            [ww + xx - yy - zz, 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), ww - xx + yy - zz, 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), ww - xx - yy + zz],
        ]


@dataclass
class Pose:
    """Rigid transform."""

    position: Vector3
    orientation: Quaternion = field(default_factory=Quaternion.identity)


@dataclass
class Mesh:
    """Triangle mesh container."""

    vertices: List[Vector3]
    faces: List[Triangle]

    def transformed(self, pose: Pose) -> "Mesh":
        rot = pose.orientation.as_rotation_matrix()
        px, py, pz = pose.position
        transformed_vertices: List[Vector3] = []
        for vx, vy, vz in self.vertices:
            rx = rot[0][0] * vx + rot[0][1] * vy + rot[0][2] * vz
            ry = rot[1][0] * vx + rot[1][1] * vy + rot[1][2] * vz
            rz = rot[2][0] * vx + rot[2][1] * vy + rot[2][2] * vz
            transformed_vertices.append((rx + px, ry + py, rz + pz))
        return Mesh(vertices=transformed_vertices, faces=list(self.faces))

    def merged(self, other: "Mesh") -> "Mesh":
        offset = len(self.vertices)
        vertices = list(self.vertices) + list(other.vertices)
        faces = list(self.faces) + [(a + offset, b + offset, c + offset) for a, b, c in other.faces]
        return Mesh(vertices=vertices, faces=faces)

    def extend(self, other: "Mesh") -> None:
        combined = self.merged(other)
        self.vertices = combined.vertices
        self.faces = combined.faces


def _circle_points(radius: float, sections: int) -> List[Vector3]:
    return [(radius * cos(2 * 3.141592653589793 * i / sections), radius * sin(2 * 3.141592653589793 * i / sections), 0.0) for i in range(sections)]


def make_cylinder(radius: float, height: float, sections: int) -> Mesh:
    """Generate a closed cylinder aligned with +Z."""

    half = height * 0.5
    bottom = -half
    top = half
    circle = _circle_points(radius, sections)
    vertices: List[Vector3] = []
    faces: List[Triangle] = []

    # Side vertices
    for x, y, _ in circle:
        vertices.append((x, y, bottom))
    for x, y, _ in circle:
        vertices.append((x, y, top))

    # Side faces
    for i in range(sections):
        j = (i + 1) % sections
        faces.append((i, j, sections + i))
        faces.append((sections + i, j, sections + j))

    # Caps
    center_bottom_index = len(vertices)
    vertices.append((0.0, 0.0, bottom))
    center_top_index = len(vertices)
    vertices.append((0.0, 0.0, top))

    for i in range(sections):
        j = (i + 1) % sections
        faces.append((center_bottom_index, j, i))
        faces.append((center_top_index, sections + i, sections + j))

    return Mesh(vertices=vertices, faces=faces)


def make_tube(inner_radius: float, outer_radius: float, height: float, sections: int) -> Mesh:
    """Annular cylinder."""

    outer = make_cylinder(outer_radius, height, sections)
    inner = make_cylinder(inner_radius, height, sections)
    # Flip inner faces for subtraction-style mesh (for STL watertightness ensure orientation)
    faces = list(outer.faces)
    vertices = list(outer.vertices)
    offset = len(vertices)
    vertices.extend(inner.vertices)
    faces.extend([(c + offset, b + offset, a + offset) for a, b, c in inner.faces])
    return Mesh(vertices=vertices, faces=faces)


def make_box(dx: float, dy: float, dz: float) -> Mesh:
    """Axis-aligned box centered at origin."""

    hx, hy, hz = dx / 2.0, dy / 2.0, dz / 2.0
    vertices = [
        (-hx, -hy, -hz),
        (hx, -hy, -hz),
        (hx, hy, -hz),
        (-hx, hy, -hz),
        (-hx, -hy, hz),
        (hx, -hy, hz),
        (hx, hy, hz),
        (-hx, hy, hz),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    return Mesh(vertices=vertices, faces=faces)


def lathe_profile(profile: Sequence[Tuple[float, float]], sections: int) -> Mesh:
    """Revolve a 2D profile (radius, z) around the Z axis."""

    vertices: List[Vector3] = []
    faces: List[Triangle] = []
    for i in range(len(profile)):
        r, z = profile[i]
        ring = _circle_points(r, sections)
        for x, y, _ in ring:
            vertices.append((x, y, z))
    rings = len(profile)
    for ring in range(rings - 1):
        for seg in range(sections):
            next_seg = (seg + 1) % sections
            a = ring * sections + seg
            b = ring * sections + next_seg
            c = (ring + 1) * sections + seg
            d = (ring + 1) * sections + next_seg
            faces.append((a, b, c))
            faces.append((c, b, d))
    return Mesh(vertices=vertices, faces=faces)


def merge_meshes(meshes: Iterable[Mesh]) -> Mesh:
    mesh_iter = iter(meshes)
    try:
        first = next(mesh_iter)
    except StopIteration:
        return Mesh([], [])
    result = Mesh(list(first.vertices), list(first.faces))
    for mesh in mesh_iter:
        result.extend(mesh)
    return result


def mesh_volume(mesh: Mesh) -> float:
    """Approximate volume using tetrahedralization from origin."""

    volume = 0.0
    for a, b, c in mesh.faces:
        ax, ay, az = mesh.vertices[a]
        bx, by, bz = mesh.vertices[b]
        cx, cy, cz = mesh.vertices[c]
        volume += (
            ax * (by * cz - bz * cy)
            - ay * (bx * cz - bz * cx)
            + az * (bx * cy - by * cx)
        ) / 6.0
    return abs(volume)


def mesh_centroid(mesh: Mesh) -> Vector3:
    """Compute centroid using volume weighting."""

    cx = cy = cz = 0.0
    volume = 0.0
    for a, b, c in mesh.faces:
        ax, ay, az = mesh.vertices[a]
        bx, by, bz = mesh.vertices[b]
        cx_v, cy_v, cz_v = mesh.vertices[c]
        vol = (
            ax * (by * cz_v - bz * cy_v)
            - ay * (bx * cz_v - bz * cx_v)
            + az * (bx * cy_v - by * cx_v)
        ) / 6.0
        centroid = (
            (ax + bx + cx_v) / 4.0,
            (ay + by + cy_v) / 4.0,
            (az + bz + cz_v) / 4.0,
        )
        cx += centroid[0] * vol
        cy += centroid[1] * vol
        cz += centroid[2] * vol
        volume += vol
    if volume == 0.0:
        return (0.0, 0.0, 0.0)
    inv = 1.0 / volume
    return (cx * inv, cy * inv, cz * inv)


def vector_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vector_scale(a: Vector3, scalar: float) -> Vector3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def translate_mesh(mesh: Mesh, offset: Vector3) -> Mesh:
    return Mesh([(vx + offset[0], vy + offset[1], vz + offset[2]) for vx, vy, vz in mesh.vertices], list(mesh.faces))


__all__ = [
    "Mesh",
    "Pose",
    "Quaternion",
    "Vector3",
    "Triangle",
    "make_cylinder",
    "make_tube",
    "make_box",
    "lathe_profile",
    "merge_meshes",
    "mesh_volume",
    "mesh_centroid",
    "vector_add",
    "vector_scale",
    "translate_mesh",
]
