import unittest
from pathgen import Rectangle, Workplace


class TestRectangle(unittest.TestCase):

    def test_intersect(self):
        rect = Rectangle((1, 1), (55.5, 33.3), (0, 0))

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



class TestWorkPlace(unittest.TestCase):

    def test_flood_fill(self):
        # test floodfill over payson park
        perimeter = [(43.679882271987395, -70.2693889874136), (43.68162231019378, -70.27141117491476),
                (43.68288076964761, -70.2725732966138), (43.68418710100531, -70.27068259077888),
                (43.68382568551194, -70.27015271143725), (43.684021633941214, -70.26986970769798),
                (43.68328573541723, -70.26871360731622), (43.6834468500671, -70.26843662493309),
                (43.68068606893511, -70.26390254378708), (43.68106927643034, -70.263511155637),
                (43.680760097846516, -70.26292106273382), (43.67989351836035, -70.26378813802968),
                (43.679122731145114, -70.26230086305941), (43.67906176450213, -70.26235505526479),
                (43.67959304316595, -70.2647214482337), (43.679366597097584, -70.26614850964243), 
                (43.67896160488272, -70.26780438258504)]
        workplace = Workplace(
            start_pos=(43.679782271987395, -70.2692889874136), 
            fov=(62.2, 48.8),   # the rpi cam 2 FOV 
            altitude=20.5, 
            perimeter=perimeter
        )

        #rects = workplace.flood_fill((0.0001, 0.0001), perimeter)
        #for r in rects:
        #    print(r)



if __name__ == "__main__":
    unittest.main()