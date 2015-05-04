from math import pi,cos,sin,acos,asin,atan,floor
import numpy as np
from irreducible_projector import IrreducibleProjector
import re
from os.path import basename
import sys

def computeNumberOfLinksFromCurvature(kappa, delta0):
        ## kappa -> N,l0
        delta1 = 0.6*delta0
        theta = pi/2
        NL = 2*sin(theta)/kappa
        lmin = 0.9*(delta0+delta1)
        N = floor(NL/lmin)
        l0 = floor(100*NL/N)/100

        if N*l0 > NL:
                print "the number of links times the length exceeds the estimate"
                sys.exit(0)

        print "Curvature",kappa," Delta0:",delta0,"=> Estimated",N,"sublinks with length",l0,". =>",N*l0,"/",NL
        return [N,l0,delta1]

def extract_numbers(txt):
        for m in re.finditer(r'[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?', txt):
                yield float(m.group(0))

def writeToFile(P, X, fnameout):

        Mpts = X.shape[0]
        print "read",Mpts,"data points along swept sphere"

        fh = open(fnameout,'w')

        fh.write('## x_0 y_0 z_0 * d_0 x_1 y_1 z_1 l_0 d_1 ... x_N y_N z_N l_{N-1} d_N\n')

        for i in range(0,Mpts):
                fh.write('%.4f %.4f %.4f %.4f %.4f ' % \
                                (X[i,0],X[i,1],X[i,2],0.0,X[i,4]))
                S = P.getSublinksPositionAtRootPosition(X[i,0:3])

                for SL in S:
                        fh.write('%.4f %.4f %.4f %.4f %.4f ' % \
                                        (SL[0],SL[1],SL[2],SL[3],SL[4]))

                fh.write('\n')
        fh.close()

def rootLinkToSublinks(fname, fnameout):

        print "reading in from",fname
        fh = open(fname,'r')
        lines = fh.read().splitlines()
        fh.close()

        M=len(lines)

        X = []
        for i in range(0,M):
                L0 = list(extract_numbers(lines[i]))
                if len(L0)==5:
                        print L0
                        X.append(L0)

        X = np.array(X)
        kappa = X[0,3]
        delta0 = X[0,4]
        [N,l0,delta1] = computeNumberOfLinksFromCurvature(kappa, delta0)

        L = l0*np.ones((N,1))
        D = delta1*np.ones((N+1,1))
        D[0]=delta0
        P = IrreducibleProjector(X[:,0:3],L,D)
        if fnameout is not None:
                writeToFile(P, X, fnameout)
        return P


def print_usage():
        print ""
        print "================================================"
        print " <<IRREDUCIBLE CURVATURE PROJECTION ALGORITHM>> "
        print "================================================"
        print ""
        print "Usage:",basename(sys.argv[0])," <INPUT-FILE> <OUTPUT-FILE>"
        print ""
        print "<INPUT-FILE> format:"
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
        print ""
        print "<OUTPUT-FILE> format:"
        print "---------------------"
        print ""
        print "    txt file containing M lines of the following format"
        print ""
        print "    X_1(1) Y_1(1) Z_1(1) L_0 D_0 "\
                        +" ... "\
                        +" X_N(1) Y_N(1) Z_N(1) L_{N-1} D_N "
        print "    ... "
        print "    X_1(M) Y_1(M) Z_1(M) L_0 D_0 "\
                        +" ... "\
                        +" X_N(M) Y_N(M) Z_N(M) L_{N-1} D_N "
        print " ... "
        print " X_M Y_M Z_M KAPPA DELTA0 "
        print ""
        print "    with:"
        print "        --- X_i(t) Y_i(t) Z_i(t)   position of sublink i at time t"
        print "        --- L_i                    length between link i-1 and i"
        print "        --- D_i                    radius of link i"
        print ""

if __name__ == "__main__":

        if len(sys.argv)!=3:
                print_usage()
                sys.exit(0)
        else:
                fname = sys.argv[1]
                fnameout = sys.argv[2]

        rootLinkToSublinks(fname, fnameout)

