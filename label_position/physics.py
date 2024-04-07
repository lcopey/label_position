from .annotate import Point, Annotation, Triangle
import numpy as np
import math


def radial_force(a, b, k):
    vector = b - a
    distance = vector.distance()
    unit_vector = vector / distance
    return unit_vector * k / distance


def force_between_triangle(left: Triangle, right: Triangle):
    # cherche la plus petite distance
    # projection des vertices de left sur les edges de right
    for vertex in left.vertices:
        for edge in right.edges:
            projection = vertex.dot(edge[1] - edge[0])
            # distance à la droite
            distance = math.sqrt((edge[0] - vertex).l2() - projection ** 2)


def compute_angle_with_radial_contraint(
        position: Point, force: Point, center: Point, radius: float
):
    new_position = position + force
    return math.atan2(new_position.y, new_position.x)


def optimize_annotation_position(points: list[Annotation], k=1, max_iter: int = 100):
    previous_norm = 1
    norm_variation = 1

    iteration = 0
    while norm_variation > 1e-2 and iteration < max_iter:
        sum_forces = Point(0, 0)
        for n, point1 in enumerate(points):
            for point2 in points[n + 1:]:
                force = radial_force(point1.text_position, point2.text_position, k)
                point1.force -= force
                point2.force += force
                sum_forces += force

        current_norm = sum_forces.distance()
        norm_variation = abs(current_norm - previous_norm)
        print(norm_variation)
        for point in points:
            next_angle = compute_angle_with_radial_contraint(
                point.text_position, point.force, point.point, point.l
            )
            point.force = Point(0, 0)
            print(f'angle {(next_angle - point.theta) * 360 / (2 * np.pi)}', )
            point.set_angle(next_angle)
        previous_norm = current_norm
        iteration += 1
