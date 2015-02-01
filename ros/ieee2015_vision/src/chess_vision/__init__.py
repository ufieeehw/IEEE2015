# from . import chess_vision
from .chess_vision import *
'''
Christine, Alan:
    When you want to add functions or modules, 
    import them as above in this __init__ file

    With this, 
    chess_vision.get_Occupancy... works

    The '.' in the above command signifies a relative import
    The * means import everything

    So here, whenever somebody imports chess_vision, the __init__ file is run,
    and we effectively import all of the objects in chess_vision.chess_vision

'''