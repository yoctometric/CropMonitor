# A module containing code that will return a set of points covering an arbitrary area

from math import tan, cos, pi, degrees, radians


class Rectangle():
    """
    A class representing a simple rectangle for use with the flood-fill algorithm
    """
    def __init__(self, center: tuple, size: tuple, index: tuple) -> None:
        self.center = center
        self.size = size
        self.yaw = 0   # used by drone for mission planning
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

        # decompose the area into a list of rectangles
        self.rectangles = self.flood_fill(size, perimeter)

        # run wavefront to get a potential field
        self.potential_field, self.grid = self.wavefront(start_pos, self.rectangles)
        # self.print_grid(self.potential_field)

        # finally, get the coverage path
        self.path = self.calc_coverage_path(self.potential_field, self.grid)


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


    def wavefront(self, home_pos: tuple, rectangles: list):
        """
        Runs the "wavefront" algorithm on the segmented workplace in order to generate a potential
        field for use in calculating a coverage path
        home_pos - the initial position of the drone

        returns the rectangles as a 2d array, and a matching potential field of ints
        """

        # First, take the list of rectangles and turn it into a 2d grid based on
        #   1. Find the min x and y 'indices' of the rectangles
        #   2. Use those values to adjust their 'index' member variable into a real 2d grid index
        #   3. Assign the rectangles to that point in the grid

        x_ind = [r.index[0] for r in rectangles]
        y_ind = [r.index[1] for r in rectangles]
        min_ind = (min(x_ind), min(y_ind))
        max_ind = (max(x_ind), max(y_ind))
        w = max_ind[0] - min_ind[0] + 1
        h = max_ind[1] - min_ind[1] + 1

        # initialize an empty grid
        grid = [[None] * h for i in range(w)]

        # now put rectangles in their proper positions
        for r in rectangles:
            x = r.index[0] - min_ind[0]
            y = r.index[1] - min_ind[1]

            # overwrite the rectangle's index here
            r.index = (x, y)

            grid[x][y] = r

        # Now for the wavefront algorithm
        # reference: https://github.com/czhanacek/python-wavefront/blob/master/wavefront.py
        #   1. Find the cell closest to the home position and make it the goal cell
        #   2. Set the cost of the goal cell and propagate out, increasing the cost of each cell
        #   3. Create path by moving to the most expensive cells first. This will result in full coverage

        # find closest cell
        closest_d = None
        goal_r = None
        for r in rectangles:
            d_squared = pow(r.center[0] - home_pos[0], 2) - pow(r.center[1] - home_pos[1], 2)  
            if closest_d is None:
                closest_d = d_squared
                goal_r = r
            elif d_squared < closest_d:
                closest_d = d_squared
                goal_r = r

        # this section re-uses a copy of the rectangle grid as a cost grid
        cost_grid = [row[:] for row in grid]
        heap = []
        new_heap = []
        x, y = goal_r.index

        # mark nodes around the goal with 3
        moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
        for move in moves:
            if self.get_grid(cost_grid, move) is None:
                continue

            cost_grid = self.set_grid(cost_grid, move, 3)
            heap.append(move)
        
        for wave in range(4, 10000):
            if len(heap) == 0:
                # last wave, no more moves needed.
                break

            while(len(heap) > 0):
                position = heap.pop()
                (x, y) = position
                moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]

                for move in moves:
                    point = self.get_grid(cost_grid, move)
                    if point is not None and isinstance(point, Rectangle):
                        cost_grid = self.set_grid(cost_grid, move, wave)
                        new_heap.append(move)

            heap = new_heap
            new_heap = []

        return cost_grid, grid

                    
    def calc_coverage_path(self, potential_field: list, cell_grid: list) -> list:
        """
        Calculates a full coverage path based on the potential field, and
        applies that path to the cell grid in order to return a path
        """
        # 1. start at the cell with the highest potential
        # 2. go to the nearest cell with the next highest potential
        # 3. repeat until goal or dead end
        #   if a dead end is hit, go backwards in the path until a new option appears
        #   to do this, keep a stack representing the head of the search that pushes and pops,
        #   and a list containing all visited cells

        # lists of tuples representing grid indices
        stack = []
        visited = []

        # find the index of the cell with the highest potential
        highest_potential = (0, 0)
        for x in range(len(potential_field)):
            for y in range(len(potential_field[0])):
                if potential_field[x][y] is None:
                    continue
                if potential_field[highest_potential[0]][highest_potential[1]] is None:
                    highest_potential = (x, y)

                if potential_field[highest_potential[0]][highest_potential[1]] < potential_field[x][y]:
                    highest_potential = (x, y)

        # the path is completed once the length of visited cells matches the number of valid cells in cell_grid
        n_cells = 0
        for row in cell_grid:
            for y in row:
                if y is not None:
                    n_cells += 1
        
        position = highest_potential
        stack.append(position)
        visited.append(position)
        while len(visited) != n_cells:
            # print(position, potential_field[position[0]][position[1]], f':\n    stack:', stack, f'\n   visited:', visited)

            # find the highest potential move from the current position
            (x, y) = position
            moves = [(x + 1, y), (x - 1, y), (x, y - 1), (x, y + 1)]
            highest_potential = None
            stuck = True
            for move in moves:
                # ignore places the search has already been
                if move in visited:
                    continue

                # ignore moves outside the workspace
                if self.get_grid(potential_field, move) is None:
                    continue

                # ignore moves trying to index outside the array
                if self.get_grid(potential_field, move) == False:
                    continue

                if highest_potential is None:
                    highest_potential = move
                    stuck = False
                if potential_field[move[0]][move[1]] > potential_field[position[0]][position[1]]:
                    highest_potential = move

            # if not stuck, append the move and continue
            if not stuck:
                position = highest_potential
                stack.append(position)
                visited.append(position)
                continue

            # however, if stuck is True, then there were no valid moves from here. So backtrack
            if len(stack) >= 1:
                position = stack.pop()
            else:
                raise AssertionError('Failed to find full coverage path')
                
        # now based on the visited indices, return the cells they correspond to
        path = []
        for index in visited:
            path.append(cell_grid[index[0]][index[1]])

        return path

    
    def photo_area_from_fov(self, fov: tuple, altitude: float, latitude) -> tuple:
        """
        Takes the camera fov (degrees) and drone altitude (meters)
        and returns the photo area coverage in decimal degrees
        """
        w = 2*altitude * tan(radians(fov[0]/2))
        h = 2*altitude * tan(radians(fov[1]/2))

        # https://stackoverflow.com/questions/25237356/convert-meters-to-decimal-degrees
        # convert meters to decimal degrees
        w = w / (111.32 * 1000 * cos(radians(latitude)))
        h = h / (111.32 * 1000 * cos(radians(latitude)))
        # print(f"Photo area (dd): ({w}, {h})")

        return (w, h)

    
    def set_grid(self, grid: list, index: tuple, value):
        """
        Sets a value in a 2d array 'grid' at index 'index'
        """
        if index[0] < 0 or index[0] >= len(grid):
            return grid

        if index[1] < 0 or index[1] >= len(grid[0]):
            return grid
        
        grid[index[0]][index[1]] = value
        return grid


    def get_grid(self, grid: list, index: tuple):
        """
        Gets the value in a 2d array 'grid' at index 'index'
        """
        # print(index, f"{(len(grid), len(grid[0]))}")

        if index[0] < 0 or index[0] >= len(grid):
            return False

        if index[1] < 0 or index[1] >= len(grid[0]):
            return False

        return grid[index[0]][index[1]]

    
    def print_grid(self, grid: list):
        """
        Prints a grid (for debugging)
        """
        for x in range(len(grid)):
            line = ''
            for y in range(len(grid[0])):
                if grid[x][y] is None:
                    line += '|--|'
                else:
                    line += f"({(grid[x][y]):02})"
            print(line)