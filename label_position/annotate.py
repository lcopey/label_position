import numpy as np

from dataclasses import dataclass, field


@dataclass
class Annotation:
    x: float
    y: float
    margin: float
    font_size: float
    text: str

# @dataclass
# class Annotation:
#     point: Point
#     theta: float
#     l: float
#     text: str
#     font_size: int
#     in_collision: bool = False
#     bbox: Triangle = None
#     text_position: Point = None
#     force: Point = field(default_factory=lambda: Point(0, 0))
#
#     def __post_init__(self):
#         self.text = str(self.text)
#         self.set_text_position()
#
#     def set_text_position(self):
#         ax = np.cos(self.theta) * self.l
#         ay = np.sin(self.theta) * self.l
#         text_position = Point(self.point.x + ax, self.point.y + ay)
#         self.text_position = text_position
#
#         right = text_position + Point(self.font_size * (len(self.text) // 2), 0)
#         left = text_position - Point(self.font_size * (len(self.text) // 2), 0)
#         if 0 < self.theta % (2 * np.pi) < np.pi:
#             self.bbox = Triangle(self.point, right, left)
#         else:
#             self.bbox = Triangle(self.point, left, right)
#
#     def set_angle(self, theta):
#         self.theta = theta
#         self.set_text_position()
#
#     def plot_bbox(self, fig, **kwargs):
#         self.bbox.plot(fig, **kwargs, showlegend=False)
#         return self
#
#     def annotate(self, fig):
#         ax = self.point.x + np.cos(self.theta) * self.l
#         ay = self.point.y + np.sin(self.theta) * self.l
#         fig.add_annotation(
#             x=self.point.x,
#             y=self.point.y,
#             text=self.text,
#             ax=ax,
#             ay=ay,
#             axref="x",
#             ayref="y",
#         )
#         return self
#
#     def collide(self, other: "Annotation"):
#         return self.bbox.line_intersect(other.bbox)
