import numpy as np

class QuinticPolynomial:
    def __init__(self, x0, dx0, ddx0, x1, dx1, ddx1, p):
        # calc coefficient of quintic polynomial
        # See jupyter notebook document for derivation of this equation.
        self.a0 = x0
        self.a1 = dx0
        self.a2 = ddx0 / 2.0

        p2 = p * p
        p3 = p * p2

        c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3
        c1 = (dx1 - ddx0 * p - dx0) / p2
        c2 = (ddx1 - ddx0) / p

        self.a3 = 0.5 * (20.0 * c0 - 8.0 * c1 + c2)
        self.a4 = (-15.0 * c0 + 7.0 * c1 - c2) / p
        self.a5 = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2

    def evaluate(self, order, p):
        if order == 0:
            return ((((self.a5 * p + self.a4) * p + self.a3) * p
                     + self.a2) * p + self.a1) * p + self.a0
        if order == 1:
            return (((5 * self.a5 * p + 4 * self.a4) * p + 3 *
                     self.a3) * p + 2 * self.a2) * p + self.a1
        if order == 2:
            return (((20 * self.a5 * p + 12 * self.a4) * p)
                    + 6 * self.a3) * p + 2 * self.a2
        if order == 3:
            return (60 * self.a5 * p + 24 * self.a4) * p + 6 * self.a3
        if order == 4:
            return 120 * self.a5 * p + 24 * self.a4
        if order == 5:
            return 120 * self.a5


class QuarticPolynomial:

    def __init__(self, x0, dx0, ddx0, dx1, ddx1, p):
        # calc coefficient of quartic polynomial

        self.a0 = x0
        self.a1 = dx0
        self.a2 = ddx0 / 2.0

        b0 = dx1 - ddx0 * p - dx0
        b1 = ddx1 - ddx0

        p2 = p * p
        p3 = p2 * p

        self.a3 = (3 * b0 - b1 * p) / (3 * p2)
        self.a4 = (-2 * b0 + b1 * p) / (4 * p3)

    def evaluate(self, order, p):
        if order == 0:
            return (((self.a4 * p + self.a3) * p + self.a2) * p
                     + self.a1) * p + self.a0
        if order == 1:
            return ((4 * self.a4 * p + 3 * self.a3) * p + 2 *
                     self.a2) * p + self.a1
        if order == 2:
            return ((12 * self.a4 * p + 6 * self.a3) * p) + 2 * self.a2
        if order == 3:
            return 24 * self.a4 * p + 6 * self.a3
        if order == 4:
            return 24 * self.a4