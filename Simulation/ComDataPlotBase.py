try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False


class ComDataPlotBase:
    def __init__(self) -> None:
        """
        This is the constructor of a class.

        Upon object creation, this sets mDataX and mDataY to None.
        """

        # Initialize mDataX to None
        self.mDataX = None 

        # Initialize mDataY to None
        self.mDataY = None


    def update(self):
        pass 

    def draw(self, ax):
        pass  