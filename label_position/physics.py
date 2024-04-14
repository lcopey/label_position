import pandas as pd

from .annotate import Annotation
import math
from dataclasses import dataclass, field
import plotly.graph_objs as go
from plotly.colors import DEFAULT_PLOTLY_COLORS


def get_arrow(at: 'Point', vector: 'Point'):
    tip = at + vector
    x = at.x, tip.x
    y = at.y, tip.y
    yield x, y
    d_theta = math.pi * 10 / 180
    l = 0.1
    base = (vector * l).rotate(d_theta)
    x = tip.x, tip.x - base.x
    y = tip.y, tip.y - base.y
    yield x, y
    base = (vector * l).rotate(-d_theta)
    x = tip.x, tip.x - base.x
    y = tip.y, tip.y - base.y
    yield x, y


@dataclass
class Point:
    x: float
    y: float

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __neg__(self):
        return Point(-self.x, -self.y)

    def __mul__(self, other):
        return Point(self.x * other, self.y * other)

    def __truediv__(self, other: float):
        return Point(self.x / other, self.y / other)

    def __eq__(self, other: 'Point') -> bool:
        return self.x == other.x and self.y == other.y

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def norm(self):
        return math.sqrt(self.l2())

    def l2(self):
        return self.x ** 2 + self.y ** 2

    def rotate(self, theta):
        return Point(self.x * math.cos(theta) - self.y * math.sin(theta),
                     self.x * math.sin(theta) + self.y * math.cos(theta))

    def plot_at(self, fig: go.Figure, at: 'Point', **kwargs):
        line_color = kwargs.pop('line_color', DEFAULT_PLOTLY_COLORS[0])
        for x, y in get_arrow(at, self):
            fig.add_scatter(x=x, y=y, showlegend=False, mode='lines', line_color=line_color, **kwargs)
        return self


@dataclass
class AABB:
    position: Point
    width: float
    height: float

    @property
    def vertices(self):
        return (
            Point(self.position.x + self.width / 2, self.position.y - self.height / 2),
            Point(self.position.x + self.width / 2, self.position.y + self.height / 2),
            Point(self.position.x - self.width / 2, self.position.y + self.height / 2),
            Point(self.position.x - self.width / 2, self.position.y - self.height / 2)
        )

    @property
    def y_max(self):
        return self.position.y + self.height / 2

    @property
    def y_min(self):
        return self.position.y - self.height / 2

    @property
    def x_max(self):
        return self.position.x + self.width / 2

    @property
    def x_min(self):
        return self.position.x - self.width / 2

    def collide(self, other: 'AABB') -> bool:
        y = (self.y_min < other.y_max < self.y_max) or (self.y_min < other.y_min < self.y_max)
        x = (self.x_min < other.x_max < self.x_max) or (self.x_min < other.x_min < self.x_max)
        return x and y

    def collision_force(self, other: 'AABB') -> Point:
        if self.collide(other):
            direction = self.position - other.position
            return direction / (direction.norm() + 1e-6)

        return Point(0, 0)

    def plot(self, fig: go.Figure, **kwargs):
        points = (*self.vertices, self.vertices[0])
        line_color = kwargs.pop('line_color', DEFAULT_PLOTLY_COLORS[0])
        for n, point in enumerate(points[:-1]):
            next_point = points[n + 1]
            x = point.x, next_point.x
            y = point.y, next_point.y

            fig.add_scatter(x=x, y=y, mode='lines', line_color=line_color, showlegend=False, **kwargs)

        return self


@dataclass
class AnnotationWithPhysics(Annotation):
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    bbox: AABB = None
    force: Point = field(default_factory=lambda: Point(0, 0))
    velocity: Point = field(default_factory=lambda: Point(0, 0))
    damping: float = 0.2
    spring: float = 0.1
    offset: float = 1
    collision: float = 5

    def __post_init__(self):
        right = abs(self.x - self.xmax)
        left = abs(self.x - self.xmin)
        top = abs(self.y - self.ymax)
        bottom = abs(self.y - self.ymin)
        d_min = min(right, left, top, bottom)
        if d_min == right:
            x, y = self.xmax, self.y
        elif d_min == left:
            x, y = self.xmin, self.y
        elif d_min == top:
            x, y = self.x, self.ymax
        else:
            x, y = self.x, self.ymin

        self.bbox = AABB(
            position=Point(x, y),
            width=(self.font_size + 2 * self.margin * self.font_size) * len(self.text),
            height=self.font_size + 2 * self.margin * self.font_size
        )

    def plot_bbox(self, fig, **kwargs):
        return self.bbox.plot(fig, **kwargs)

    def collide(self, other: 'AnnotationWithPhysics'):
        force = self.bbox.collision_force(other.bbox)
        self.force += force * self.collision
        other.force -= force * other.collision

    def spring_force(self):
        direction = self.bbox.position - Point(self.x, self.y)
        norm = direction.norm()
        if norm == 0:
            unit = Point(0, 1)
        else:
            unit = direction / norm
        magnitude = norm - self.offset * self.font_size
        magnitude = 2 * magnitude if magnitude < 0 else magnitude**2

        return -unit * magnitude

    def apply_forces(self, dt: float):
        self.force += self.spring_force()
        self.velocity += self.force * dt
        displacement = self.velocity * dt
        self.bbox.position += displacement
        self.velocity *= (1 - self.damping)
        return displacement

    def reset_force(self):
        self.force = Point(0, 0)

    def step(self, dt: float = 0.1):
        displacement = self.apply_forces(dt)
        self.reset_force()
        return displacement

    def annotate(self, fig: go.Figure):
        fig.add_annotation(
            x=self.x,
            y=self.y,
            ax=self.bbox.position.x,
            ay=self.bbox.position.y,
            axref='x',
            ayref='y',
            text=self.text
        )

# class DeterminantResult(Enum):
#     left = auto()
#     right = auto()
#     colinear = auto()
#
#
# def determinant(left: Point, right: Point):
#     return left.x * right.y - left.y * right.x
#
#
# def _relative_position(vector: tuple[Point, Point], other: Point):
#     a, b = vector
#     d = b - a
#     t = other - a
#     result = determinant(d, t)
#     if result > 0:
#         return DeterminantResult.left
#     elif result < 0:
#         return DeterminantResult.right
#     else:
#         return DeterminantResult.colinear
#
#
# def line_intersect(edge: tuple[Point, Point], other: tuple[Point, Point]):
#     """
#     judge if line (v1,v2) intersects with line(v3,v4)
#     """
#     v1, v2 = edge
#     v3, v4 = other
#     d = (v4.y - v3.y) * (v2.x - v1.x) - (v4.x - v3.x) * (v2.y - v1.y)
#     u = (v4.x - v3.x) * (v1.y - v3.y) - (v4.y - v3.y) * (v1.x - v3.x)
#     v = (v2.x - v1.x) * (v1.y - v3.y) - (v2.y - v1.y) * (v1.x - v3.x)
#     if d < 0:
#         u, v, d = -u, -v, -d
#     return (0 <= u <= d) and (0 <= v <= d)
#
#
# @dataclass
# class Triangle:
#     """Les points doivent être dans l'ordre
#     de manière à ce que la gauche représente l'intérieur du triangle"""
#
#     a: Point
#     b: Point
#     c: Point
#
#     @property
#     def vertices(self):
#         return (self.a, self.b, self.c)
#
#     @property
#     def edges(self):
#         return ((self.a, self.b), (self.b, self.c), (self.c, self.a))
#
#     def point_in_triangle(self, other: "Triangle"):
#         # vérifie si l'un des points de other se trouve
#         # tous à gauche de toutes les edges de self
#         for vertex in other.vertices:
#             if all(
#                     [
#                         _relative_position(edge, vertex) == DeterminantResult.left
#                         for edge in self.edges
#                     ]
#             ):
#                 return True
#         return False
#
#     def line_intersect(self, other: "Triangle"):
#         for other_edge in other.edges:
#             for self_edge in self.edges:
#                 if line_intersect(other_edge, self_edge):
#                     return True
#         else:
#             return False
#
#     def plot(self, fig, line_color=None, **kwargs):
#         x = [vertex.x for vertex in [*self.vertices, self.vertices[0]]]
#         y = [vertex.y for vertex in [*self.vertices, self.vertices[0]]]
#         text = ["a", "b", "c", ""]
#         fig.add_scatter(
#             x=x,
#             y=y,
#             text=text,
#             line_color=line_color
#             if line_color is not None
#             else DEFAULT_PLOTLY_COLORS[0],
#             mode="lines+text",
#             **kwargs
#         )
#         return self
#
#
# def radial_force(a, b, k):
#     vector = b - a
#     distance = vector.distance()
#     unit_vector = vector / distance
#     return unit_vector * k / distance
#
#
# def force_between_triangle(left: Triangle, right: Triangle):
#     # cherche la plus petite distance
#     # projection des vertices de left sur les edges de right
#     for vertex in left.vertices:
#         for edge in right.edges:
#             projection = vertex.dot(edge[1] - edge[0])
#             # distance à la droite
#             distance = math.sqrt((edge[0] - vertex).l2() - projection ** 2)
#
#
# def compute_angle_with_radial_constraint(
#         position: Point, force: Point, center: Point, radius: float
# ):
#     new_position = position + force
#     return math.atan2(new_position.y, new_position.x)
#
#
# def optimize_annotation_position(points: list[Annotation], k=1, max_iter: int = 100):
#     previous_norm = 1
#     norm_variation = 1
#
#     iteration = 0
#     while norm_variation > 1e-2 and iteration < max_iter:
#         sum_forces = Point(0, 0)
#         for n, point1 in enumerate(points):
#             for point2 in points[n + 1:]:
#                 force = radial_force(point1.text_position, point2.text_position, k)
#                 point1.force -= force
#                 point2.force += force
#                 sum_forces += force
#
#         current_norm = sum_forces.distance()
#         norm_variation = abs(current_norm - previous_norm)
#         print(norm_variation)
#         for point in points:
#             next_angle = compute_angle_with_radial_constraint(
#                 point.text_position, point.force, point.point, point.l
#             )
#             point.force = Point(0, 0)
#             print(f'angle {(next_angle - point.theta) * 360 / (2 * np.pi)}', )
#             point.set_angle(next_angle)
#         previous_norm = current_norm
#         iteration += 1
