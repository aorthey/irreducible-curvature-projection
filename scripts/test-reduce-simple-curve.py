import time
import sys
import numpy as np
import pickle as pk
from irreducible_sublink_projector import rootLinkToSublinks

if __name__ == '__main__':
        fname = "../data/spheretraj.txt"
        fnameout = "../data/spheretraj-sublinks.txt"
        fname = "../data/TRO_path.txt"
        fnameout = "../data/TRO_path_full2.txt"
        rootLinkToSublinks(fname, fnameout)

