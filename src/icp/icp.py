#!/usr/bin/env python3

import math
import numpy as np
import sys
import random


class ScanMathcerICP(object):
    """ Scan Mathing with ICP algorithm
    Attributes:
        reference_points(numpy.ndarray):
            List of coordinate pair of reference scan
        current_points(numpy.ndarray):
            List of coordinate pair of current scan
        epsilon(double):
            Threshold of residual
        max_iterations(int):
            Maximum iteration number
        distance_threshold(double):
            Distance threshold to be ignored in the association
        cost_delta_threshold(double):
            Threshold value of amount of change of cost
    """

    def __init__(self):
        self.reference_points = None
        self.current_points = None

        self.epsilon = 1e-4
        self.max_iterations = 100
        self.distance_threshold = 5.0
        self.cost_delta_threshold = 1e-3

    def optimize(self, reference_points, current_points):
        """
        Optimize robot pose

        Args:
            referene_points(numpy.ndarray):
                Reference scan points
            current_scan(numpy.ndarray):
                Current scan points

        Returns:
            numpy.ndarray:
                Estimated residual vector
        """
        # Register scans
        self.reference_points = reference_points
        self.current_points = current_points

        # Estimated residual vector
        estimated_residual = np.array([0.0, 0.0, 0.0])

        # Copy current scan for computation
        optimizing_points = self.current_points

        # Numeric step for numerical integration
        linear_step = 1e-3
        angular_step = 1e-3

        # coefficient for steepest descent
        sd_coef = 1e-2

        # Cost
        current_cost = sys.float_info.max
        previous_cost = 0.0

        # Optimization count
        iterations = 0

        # Initial cost computation
        indexes = self.associate(optimizing_points)
        current_cost = self.score(optimizing_points, indexes)

        while (abs(previous_cost - current_cost) > self.cost_delta_threshold):
            iterations += 1
            print(f"{iterations} times iteration...")
            print(estimated_residual)

            # Update previous cost
            previous_cost = current_cost

            # Numerical partial differencial
            # x axis direction
            dx_transformed_points = self.transform(
                    optimizing_points,
                    (estimated_residual + np.array([linear_step, 0.0, 0.0]))
                    )
            dx_cost = self.score(
                    dx_transformed_points,
                    self.associate(dx_transformed_points)
                    )
            dx = (dx_cost - current_cost) / linear_step

            # y axis direction
            dy_transformed_points = self.transform(
                    optimizing_points,
                    (estimated_residual + np.array([0.0, linear_step, 0.0]))
                    )
            dy_cost = self.score(
                    dy_transformed_points,
                    self.associate(dy_transformed_points)
                    )
            dy = (dy_cost - current_cost) / linear_step

            # yaw axis direction
            dtheta_transformed_points = self.transform(
                    optimizing_points,
                    (estimated_residual + np.array([0.0, 0.0, angular_step]))
                    )
            dtheta_cost = self.score(
                    dtheta_transformed_points,
                    self.associate(dtheta_transformed_points)
                    )
            dtheta = (dtheta_cost - current_cost) / angular_step

            # Update residual vector
            estimated_residual = estimated_residual + np.array([-sd_coef * dx,
                                                                -sd_coef * dy,
                                                                -sd_coef * dtheta])

            # Update current cost
            current_cost = self.score(
                    self.transform(optimizing_points, estimated_residual),
                    self.associate(
                        self.transform(optimizing_points, estimated_residual)
                        )
                    )
            print(current_cost)

        return estimated_residual

    def associate(self, current_points):
        """
        Data association between reference scan to current scan

        Args:
            current_points(numpy.ndarray):

        Returns:
            numpy.ndarray:
                List which containing the indexes of the
                reference scan corresponding to the current scan.
        """
        corresponded_indexes = []

        for (i, cp) in enumerate(current_points):
            minimum_distance = sys.float_info.max
            corresponded_index = None

            for (j, rp) in enumerate(self.reference_points):
                distance = np.linalg.norm(cp - rp)

                if (distance <= self.distance_threshold
                        and distance < minimum_distance):
                    minimum_distance = distance
                    corresponded_index = j

            corresponded_indexes.append(corresponded_index)

        return np.array(corresponded_indexes)

    def score(self, current_points, corresponded_indexes):
        """
        Compute scan matching score

        Args:
            current_points(numpy.ndarray):
                Current scan points
            corresponded_index(numpy.ndarray):
                List which containing the indexes of the
                reference scan corresponding to the current scan, which
                is the argument of this function.

        Returns:
            double:
                Scan matching score.
                Sum of the squares of the distance between each of
                corresponded points
        """
        score = 0.0

        for (i, index) in enumerate(corresponded_indexes):
            delta = current_points[i] - self.reference_points[index]

            score += (delta[0]**2 + delta[1]**2)

        return score

    def transform(self, points, residual):
        """
        Transform the coordinates of each points using the residual
        transform vector.

        Args:
            points(numpy.ndarray):
                Points for transformation
            residual(numpy.ndarray):
                Residual Vector

        Returns:
            numpy.ndarray:
                Transformed points
        """

        transformed_points = [[
            residual[0] + p[0]*np.cos(residual[2]) - p[1]*np.sin(residual[2]),
            residual[1] + p[0]*np.sin(residual[2]) + p[1]*np.cos(residual[2])
            ] for p in points]

        return np.array(transformed_points)


def main():
    num_points = 10
    field_length = 5.0
    true_residual = [0.5, 2.0, math.pi/6]

    # Generate reference points
    current_points = [
        [(random.random() - 0.5) * field_length,
         (random.random() - 0.5) * field_length] for i in range(num_points)]

    # Generate current points
    reference_points = [
            [
                true_residual[0]
                + r[0] * math.cos(true_residual[2])
                - r[1] * math.sin(true_residual[2]),
                true_residual[1]
                + r[0] * math.sin(true_residual[2])
                + r[1] * math.cos(true_residual[2])
            ] for r in current_points
            ]

    scanmatcher = ScanMathcerICP()

    estimated_residual = scanmatcher.optimize(
            np.array(reference_points),
            np.array(current_points)
            )

    print(f"True residual is {true_residual}")
    print(f"Estimated residual is {estimated_residual}")


if __name__ == "__main__":
    main()
