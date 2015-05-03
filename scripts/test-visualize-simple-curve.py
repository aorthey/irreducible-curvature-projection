import time
import sys
import numpy as np
import pickle as pk
from irreducible_sublink_projector import rootLinkToSublinks

if __name__ == '__main__':
        fname = "../data/spheretraj.txt"
        P = rootLinkToSublinks(fname, None)
        deltaT = 0.001
        P.visualizeLinearLinkageProjection(deltaT)

