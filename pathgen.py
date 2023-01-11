# A module containing code that will return a set of points covering an arbitrary area

from math import tan, cos, pi, degrees, radians


class Rectangle():
    """
    A class representing a simple rectangle for use with the flood-fill algorithm
    """
    def __init__(self, center: tuple, size: tuple, index: tuple) -> None:
        self.center = center
        self.size = size
        self.index = index  # The integer tuple index of the rectangle in the decomposed grid
        self.border = False # Is this rectangle intersecting the perimeter


    def intersects(self, a: tuple, b: tuple) -> bool:
        """
        Returns whether a line segment intersects or is contained within this rectangle
        https://stackoverflow.com/a/16203792
        """

        # get the corners of the rectangle
        corners = [
            (self.center[0] + self.size[0]/2, self.center[1] + self.size[1]/2),
            (self.center[0] - self.size[0]/2, self.center[1] + self.size[1]/2),
            (self.center[0] - self.size[0]/2, self.center[1] - self.size[1]/2),
            (self.center[0] + self.size[0]/2, self.center[1] - self.size[1]/2),
        ]

        # check whether an endpoint of the test line is completely contained within the rectangle
        if (a[0] < corners[0][0] and a[0] > corners[2][0] and a[1] < corners[0][1] and a[1] > corners[2][1]):
            return True

        # iterate through all line segments composing the rectangle and check for intersections
        for i in range(0, 4):
            j = i + 1
            if j >= 4:
                j = 0
            
            # parameterize the rectangle side
            u0 = corners[i]                                     # (x00, y00)
            v0 = (corners[j][0]-u0[0], corners[j][1]-u0[1])     # (x01, y01)

            # parameterize the line segment that we are testing for intersection
            u1 = a                                              # (x10, y10)
            v1 = (b[0]-u1[0], b[1]-u1[1])                       # (x11, y11)

            # do some math (see the SO link)
            det = v1[0] * v0[1] - v0[0] * v1[1]
            if det == 0: # case where the lines are parallel
                return False

            s = 1/det * (      (u0[0]-u1[0]) * v0[1] - (u0[1]-u1[1]) * v0[0])
            t = 1/det * -1*(-1*(u0[0]-u1[0]) * v1[1] + (u0[1]-u1[1]) * v1[0])

            # if both s and t are between 0 and 1, the lines intersect!
            if s > 0 and s < 1 and t > 0 and t < 1:
                return True
        
        return False


    def overlaps_polygon(self, polygon: list) -> bool:
        """
        Returns true if the rectangle intersects or contains the polygon
        """
        for i in range(len(polygon)):
            j = i + 1
            j = j % len(polygon)
            intersects = self.intersects(polygon[i], polygon[j])
            if intersects:
                return True
        
        return False


    def __str__(self) -> str:
        return f"({self.center[0]}, {self.center[1]})"



    def spread(self):
        """
        Returns four new rectangles adjacent to each face of this one
        """
        return [
            Rectangle((self.center[0]-self.size[0], self.center[1]), self.size, (self.index[0]-1, self.index[1])),
            Rectangle((self.center[0]+self.size[0], self.center[1]), self.size, (self.index[0]+1, self.index[1])),
            Rectangle((self.center[0], self.center[1]-self.size[1]), self.size, (self.index[0], self.index[1]-1)),
            Rectangle((self.center[0], self.center[1]+self.size[1]), self.size, (self.index[0], self.index[1]+1)),
        ]



class Workplace():
    """
    A class representing the Workplace to be used in workplace sampling for path generation
    Reference: https://core.ac.uk/download/pdf/74476273.pdf
    """

    def __init__(self, start_pos: tuple, fov: tuple, altitude: float, perimeter: list):
        """
        Segments the workplace grid based on the FOV and altitude the drone will fly at
        Uses Approximate Cellular Decomposition to do so
        """
        
        # get the width and height of the capture rectangles from fov in meters
        size = self.photo_area_from_fov(fov, altitude, start_pos[1])

        # decompose the area into a grid
        self.grid = self.flood_fill(size, perimeter)

        # run wavefront to get a full coverage path
        self.path = self.wavefront(start_pos)


    def flood_fill(self, size: tuple, perimeter: list) -> list:
        """
        Algorithm which fills a polygon with equally spaced, equally sized rectangles
        """
        
        # 1. Find the centerpoint of the polygon
        center = [0, 0]
        for p in perimeter:
            center[0] += p[0]
            center[1] += p[1]
        
        center[0] = center[0] / len(perimeter)
        center[1] = center[1] / len(perimeter)

        # 2. initialize the center rectangle
        unspent = [Rectangle(center, size, (0, 0))]
        spent = []

        # handle case where initial rectangle already fully encompasses perimeter
        if unspent[0].overlaps_polygon(perimeter):
            spent = [unspent[0]]
            unspent = []

        # 3. While "unspent" rectangles exist in the list:
        #   for each unspent rectangle, replicate to adjacent tiles and mark as spent
        while len(unspent) > 0:
            new_unspent = []

            for rect in unspent:
                # spread the rectangle
                new_rects = rect.spread()

                for new_rect in new_rects:
                    # check to make sure the new rects are on unoccupied spaces
                    overlaps = False

                    for r in unspent + spent + new_unspent:
                        if r.index == new_rect.index:
                            overlaps = True
                            break
                    if overlaps:
                        continue    # if the rect is on an occupied space, dont bother dealing with it
                    
                    # check to see if the rect is overlapping a polygon edge
                    new_rect.border = new_rect.overlaps_polygon(perimeter)

                    # if the rect does not intersect but the rect that spawned it does, then this rect is outside the polygon
                    if (not new_rect.border) and rect.border:
                        continue # don't add it

                    new_unspent.append(new_rect)

                # move the rect we just worked on into the spent category
                spent.append(rect)
            
            # overwrite the unspent array with the new unspent array
            unspent = new_unspent

        return spent


    def wavefront(self, home_pos: tuple):
        """
        Runs the "wavefront" algorithm on the segmented workplace in order to generate a full coverage
        flight path.
        home_pos - the initial position of the drone
        """

    
    def wavefront_cubic(self):
        # TODO: implement if interested
        """
        Generates a path using the wavefront algorithm and smooths it with cubic bezier curves
        """

    
    def photo_area_from_fov(self, fov: tuple, altitude: float, latitude) -> tuple:
        """
        Takes the camera fov (degrees) and drone altitude (meters)
        and returns the photo area coverage in decimal degrees
        """
        w = 2*altitude * tan(radians(fov[0]/2))
        h = 2*altitude * tan(radians(fov[1]/2))
        print(f"Photo area (m): ({w}, {h})")

        # https://stackoverflow.com/questions/25237356/convert-meters-to-decimal-degrees
        # convert meters to decimal degrees
        w = w / (111.32 * 1000 * cos(radians(latitude)))
        h = h / (111.32 * 1000 * cos(radians(latitude)))
        print(f"Photo area (dd): ({w}, {h})")

        return (w, h)