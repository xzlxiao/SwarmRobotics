try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False


class ComDataPlotBase:
    def __init__(self) -> None:
        self.mDataX = None 
        self.mDataY = None

    def update(self):
        pass 

    def draw(self, ax):
        pass  
