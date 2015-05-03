#Irreducible Curvature Projection

This module implements the curvature projection algorithm, described in the paper "Irreducible Motion
Planning by Exploiting Linear Linkage Structures"

## Dependencies

 * numpy
 * scipy
 * matplotlib

## Definition Linear Linkage

A linear linkage is a mechanical system with N+1 links, arranged as
L0->L1->L2->...->LN, connected by spherical joints

![Linear Linkage](https://github.com/orthez/irreducible-curvature-projection/raw/master/images/hierarchy-chain.png "Linear Linkage")

L0 is called the **root link**

L1,L2,...,LN are called the **sublinks**

## Input

To run the algorithm, we need the following data

 * the trajectory of the root link L0 as discrete samples 
 * length L between links
 * radius D of all links

## Output

 * the joint configuration at each instance $t \in [0,1]$

## Fast Start

An example trajectory can be found in data/spheretraj.txt, which contains a list
of discrete x,y,z positions of a spherical root link, together with its radius
and its curvature. 

    python test-reduce-simple-curve.py
    python test-visualize-simple-curve.py


## Projection from given file

A simple trajectory is described in data/spheretraj.txt; To visualize the sublinks along the trajectory
    
    python scripts/projection_visualizer.py data/spheretraj.txt
    
to compute the sublinks along the trajectory and write them to the file data/spheretraj-sublinks.txt invoke:

    python scripts/irreducible_sublink_projector.py data/spheretraj.txt data/spheretraj-sublinks.txt
    
##Irreducible Motion Planning for Linear Linkages

The core curvature projection algorithm can be found in the python module called IrreducibleProjector. 

An example for the left arm of the humanoid robot HRP-2. We have a linear
linkage structure L0->L1->L2->L3, with l0=0.25m and delta_0 = 0.08m. 
![Arm HRP-2](https://github.com/orthez/irreducible-curvature-projection/raw/master/images/arm_linear_linkage.png "Arm as Linear Linkage")

Let X be the discrete set of locations of L0 over time. Then we can compute the
irreducible configurations in a python script as

    L = np.array((0.25,0.25,0.25))
    D = np.array((0.08,0.08,0.08,0.08))
    P = IrreducibleProjector(X,L,D)
    
To compute a specific configuration at a specific time t0 do

    P.getJointAnglesAtT(t0)
    
To visualize the configuration at t0

    P.plotLinearLinkageAtT(t0)
    
To compute the configurations over the complete trajectory of L0 and display each configuration for 0.0001s do

    P.visualizeLinearLinkageProjection(0.0001)

## Problems

  * Internally, we interpolate the samples by a spline representation. In some cases where the samples are not dense enough, the interpolation might be not optimal. In this case you have the possibility to change the splinePrecision variable in irreducible_projector.py, which controls the precision of the spline interpolation
