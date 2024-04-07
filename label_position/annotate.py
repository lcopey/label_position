import numpy as np
from plotly.colors import DEFAULT_PLOTLY_COLORS

import math
from dataclasses import dataclass, field
from enum import Enum, auto

__all__ = ['Point', 'DeterminantResult', 'Triangle', 'Annotation']


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

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def distance(self):
        return math.sqrt(self.l2())

    def l2(self):
        return self.x ** 2 + self.y ** 2


class DeterminantResult(Enum):
    left = auto()
    right = auto()
    colinear = auto()


def determinant(left: Point, right: Point):
    return left.x * right.y - left.y * right.x


def _relative_position(vector: tuple[Point, Point], other: Point):
    a, b = vector
    d = b - a
    t = other - a
    result = determinant(d, t)
    if result > 0:
        return DeterminantResult.left
    elif result < 0:
        return DeterminantResult.right
    else:
        return DeterminantResult.colinear


def line_intersect(edge: tuple[Point, Point], other: tuple[Point, Point]):
    """
    judge if line (v1,v2) intersects with line(v3,v4)
    """
    v1, v2 = edge
    v3, v4 = other
    d = (v4.y - v3.y) * (v2.x - v1.x) - (v4.x - v3.x) * (v2.y - v1.y)
    u = (v4.x - v3.x) * (v1.y - v3.y) - (v4.y - v3.y) * (v1.x - v3.x)
    v = (v2.x - v1.x) * (v1.y - v3.y) - (v2.y - v1.y) * (v1.x - v3.x)
    if d < 0:
        u, v, d = -u, -v, -d
    return (0 <= u <= d) and (0 <= v <= d)


@dataclass
class Triangle:
    """Les points doivent être dans l'ordre
    de manière à ce que la gauche représente l'intérieur du triangle"""

    a: Point
    b: Point
    c: Point

    @property
    def vertices(self):
        return (self.a, self.b, self.c)

    @property
    def edges(self):
        return ((self.a, self.b), (self.b, self.c), (self.c, self.a))

    def point_in_triangle(self, other: "Triangle"):
        # vérifie si l'un des points de other se trouve
        # tous à gauche de toutes les edges de self
        for vertex in other.vertices:
            if all(
                    [
                        _relative_position(edge, vertex) == DeterminantResult.left
                        for edge in self.edges
                    ]
            ):
                return True
        return False

    def line_intersect(self, other: "Triangle"):
        for other_edge in other.edges:
            for self_edge in self.edges:
                if line_intersect(other_edge, self_edge):
                    return True
        else:
            return False

    def plot(self, fig, line_color=None, **kwargs):
        x = [vertex.x for vertex in [*self.vertices, self.vertices[0]]]
        y = [vertex.y for vertex in [*self.vertices, self.vertices[0]]]
        text = ["a", "b", "c", ""]
        fig.add_scatter(
            x=x,
            y=y,
            text=text,
            line_color=line_color
            if line_color is not None
            else DEFAULT_PLOTLY_COLORS[0],
            mode="lines+text",
            **kwargs
        )
        return self


@dataclass
class Annotation:
    point: Point
    theta: float
    l: float
    text: str
    font_size: int
    in_collision: bool = False
    bbox: Triangle = None
    text_position: Point = None
    force: Point = field(default_factory=lambda: Point(0, 0))

    def __post_init__(self):
        self.text = str(self.text)
        self.set_text_position()

    def set_text_position(self):
        ax = np.cos(self.theta) * self.l
        ay = np.sin(self.theta) * self.l
        text_position = Point(self.point.x + ax, self.point.y + ay)
        self.text_position = text_position

        right = text_position + Point(self.font_size * (len(self.text) // 2), 0)
        left = text_position - Point(self.font_size * (len(self.text) // 2), 0)
        if 0 < self.theta % (2 * np.pi) < np.pi:
            self.bbox = Triangle(self.point, right, left)
        else:
            self.bbox = Triangle(self.point, left, right)

    def set_angle(self, theta):
        self.theta = theta
        self.set_text_position()

    def plot_bbox(self, fig, **kwargs):
        self.bbox.plot(fig, **kwargs, showlegend=False)
        return self

    def annotate(self, fig):
        ax = self.point.x + np.cos(self.theta) * self.l
        ay = self.point.y + np.sin(self.theta) * self.l
        fig.add_annotation(
            x=self.point.x,
            y=self.point.y,
            text=self.text,
            ax=ax,
            ay=ay,
            axref="x",
            ayref="y",
        )
        return self

    def collide(self, other: "Annotation"):
        return self.bbox.line_intersect(other.bbox)
