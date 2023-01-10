# A module containing code that will return a set of points covering an arbitrary area


class Workplace():
    """
    A class representing the Workplace to be used in workplace sampling for path generation
    Reference: https://core.ac.uk/download/pdf/74476273.pdf
    """

    def __init__(fov: tuple, altitude: float, perimeter: list):
        """
        Segments the workplace grid based on the FOV and altitude the drone will fly at
        Uses Approximate Cellular Decomposition to do so
        """
        pass


    def flood_fill(size: tuple, perimeter: list):
        """
        Algorithm which fills a polygon with equally spaced, equally sized rectangles
        """
        



    def wavefront():
        """
        Runs the "wavefront" algorithm on the segmented workplace in order to generate a full coverage
        flight path.
        """

    
    def wavefront_cubic():
        # TODO: implement if interested
        """
        Generates a path using the wavefront algorithm and smooths it with cubic bezier curves
        """