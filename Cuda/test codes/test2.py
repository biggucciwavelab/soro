# -*- coding: utf-8 -*-
"""
Created on Wed Aug 14 18:41:56 2019

@author: qiyua
"""
import numpy as np
import cupy as cp
import timeit

start = timeit.default_timer()



def LCPSolve(M,q, pivtol=1e-8): # pivtol = smallest allowable pivot element
    rayTerm = False
    loopcount = 0
    if (q >= 0.).all(): # Test missing in Rob Dittmar's code
        # As w - Mz = q, if q >= 0 then w = q and z = 0
        w = q
        z = cp.zeros_like(q)
        retcode = 0.
    else:
        dimen = M.shape[0] # number of rows
        # Create initial tableau
        tableau = cp.hstack([cp.eye(dimen), -M, -cp.ones((dimen, 1)), cp.asarray(np.asmatrix(q).T)])
        # Let artificial variable enter the basis
        basis = list(range(dimen)) # basis contains a set of COLUMN indices in the tableau 
        locat = int(cp.argmin(tableau[:,2*dimen+1])) # row of minimum element in column 2*dimen+1 (last of tableau)
        basis[locat] = int(2*dimen) # replace that choice with the row 
        cand = locat + dimen
        pivot = tableau[locat,:]/tableau[locat,2*dimen]
        tableau -= tableau[:,2*dimen:2*dimen+1]*pivot # from each column subtract the column 2*dimen, multiplied by pivot 
        tableau[locat,:] = pivot # set all elements of row locat to pivot
        # Perform complementary pivoting
        oldDivideErr = seterr(divide='ignore')['divide'] # suppress warnings or exceptions on zerodivide inside numpy
        while amax(basis) == 2*dimen:
            loopcount += 1
            eMs = tableau[:,cand]    # Note: eMs is a view, not a copy! Do not assign to it...
            missmask = eMs <= 0.
            quots = tableau[:,2*dimen+1] / eMs # sometimes eMs elements are zero, but we suppressed warnings...
            quots[missmask] = Inf # in any event, we set to +Inf elements of quots corresp. to eMs <= 0. 
            locat = int(cp.argmin(quots))
            if abs(eMs[locat]) > pivtol and not missmask.all(): # and if at least one element is not missing 
                # reduce tableau
                pivot = tableau[locat,:]/tableau[locat,cand]
                tableau -= tableau[:,cand:cand+1]*pivot 
                tableau[locat,:] = pivot
                oldVar = basis[locat]
                # New variable enters the basis
                basis[locat] = cand
                # Select next candidate for entering the basis
                if oldVar >= dimen:
                    cand = oldVar - dimen
                else:
                    cand = oldVar + dimen
            else:
                rayTerm = True
                break
        seterr(divide=oldDivideErr) # restore original handling of zerodivide in Numpy
        # Return solution to LCP
        vars = cp.zeros(2*dimen+1)
        vars[basis] = tableau[:,2*dimen+1]
        w = vars[:dimen]
        z = vars[dimen:2*dimen]    
        retcode = vars[2*dimen]
    # end if (q >= 0.).all() 
    
    if rayTerm:
        retcode = (2, retcode, loopcount)  # ray termination
    else:
        retcode = (1, retcode, loopcount)  # success
    return (w, z, retcode)

M1=np.load('A1.npy')
q1=np.load('B1.npy')

M2=cp.array(M1)
q2=cp.array(q1)

z2,w2,retcode2=LCPSolve(M2,q2)

stop = timeit.default_timer()

print('M2: %s' % (M2))
print('q2: %s' % (q2))

print('w2: %s' % (w2))
print('z2: %s' % (z2))
print(retcode2)

print('Time: ', stop - start) 