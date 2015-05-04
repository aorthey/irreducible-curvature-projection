import time
import sys
import numpy as np
import pickle as pk
from irreducible_sublink_projector import rootLinkToSublinks

if __name__ == '__main__':
        #fname = "../data/TRO_path.txt"
        fname = "../data/TRO_path.txt"
        fname = "../data/spheretraj.txt"
        P = rootLinkToSublinks(fname, None)
        P.plotLinearLinkageAtGoal()
        #P.plotLinearLinkageAtTLimits(0.8,-3,0,1,5,-1,1)
