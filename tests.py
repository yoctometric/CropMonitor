import unittest
from pathgen import Rectangle


class TestRectangle(unittest.TestCase):

    def test_intersect(self):
        rect = Rectangle((1, 1), (55.5, 33.3))

        # test segment contained by rectangle
        intersects = rect.intersects((-3, 3.5), (2.5, 2.7))
        self.assertTrue(intersects)

        # various intersections
        intersects = rect.intersects((-100, 0), (-15, 1))
        self.assertTrue(intersects)

        intersects = rect.intersects((55, 48), (-100, -100))
        self.assertTrue(intersects)

        # false cases
        intersects = rect.intersects((0, 101), (101, 101))
        self.assertFalse(intersects)



if __name__ == "__main__":
    unittest.main()