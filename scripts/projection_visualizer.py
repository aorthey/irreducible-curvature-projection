#/usr/bin/env python
import time
import sys
import numpy as np
from irreducible_sublink_projector import rootLinkToSublinks
from os.path import basename

def print_usage():
        print ""
        print "================================================"
        print " <<IRREDUCIBLE CURVATURE VISUALIZATION>> "
        print "================================================"
        print ""
        print "Usage:",basename(sys.argv[0])," <ROOT-LINK-POSITION-FILE>"
        print ""
        print "<ROOT-LINK-POSITION-FILE> format:"
        print "---------------------"
        print ""
        print "    txt file containing M lines of the following format"
        print ""
        print "    X_1 Y_1 Z_1 KAPPA DELTA0 "
        print "    ... "
        print "    X_M Y_M Z_M KAPPA DELTA0 "
        print ""
        print "    with:"
        print "        --- X_i Y_i Z_i   position of root link"
        print "        --- KAPPA         maximum curvature"
        print "        --- DELTA0        radius of the root link"


if __name__ == "__main__":
        if len(sys.argv)!=2:
                print_usage()
                sys.exit(0)
        else:
                froot = sys.argv[1]

        P = rootLinkToSublinks(froot, None)
        deltaT = 0.001
        P.visualizeLinearLinkageProjection(deltaT)
