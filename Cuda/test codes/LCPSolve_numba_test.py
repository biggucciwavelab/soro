''' LCPSolve(M,q): procedure to solve the linear complementarity problem:

       w = M z + q
       w and z >= 0
       w'z = 0

   The procedure takes the matrix M and vector q as arguments.  The
   procedure has three returns.  The first and second returns are
   the final values of the vectors w and z found by complementary
   pivoting.  The third return is a 2 by 1 vector.  Its first
   component is a 1 if the algorithm was successful, and a 2 if a
   ray termination resulted.  The second component is the value of
   the artificial variable upon termination of the algorithm.
   The third component is the number of iterations performed in the
   outer loop.
 
   Derived from: http://www1.american.edu/academic.depts/cas/econ/gaussres/optimize/quadprog.src
   (original GAUSS code by Rob Dittmar <dittmar@stls.frb.org> )

   Lemke's Complementary Pivot algorithm is used here. For a description, see:
   http://ioe.engin.umich.edu/people/fac/books/murty/linear_complementarity_webbook/kat2.pdf

Copyright (c) 2010 Rob Dittmar, Enzo Michelangeli and IT Vision Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

'''
   
#from numba import cuda
from numpy import *
import numpy as np
from numba import vectorize, float64
import timeit

start = timeit.default_timer()

class baseSolver:
    def __init__(self):pass
    __name__ = 'Undefined. If you are a user and got the message, inform developers please.'
    __license__ = "Undefined. If you are a user and got the message, inform developers please."
    __authors__ = "Undefined. If you are a user and got the message, inform developers please."
    __alg__ = "Undefined"
    __solver__ = "Undefined. If you are a user and got the message, inform developers please."
    __homepage__ = 'Undefined. Use web search'
    __info__ = 'None'
    _requiresBestPointDetection = False
    _requiresFiniteBoxBounds = False

    """ useStopByException should be turned to False
    for those solvers where passing exceptions through C/Fortran code is buggy
    and cannot be catched by OO
    """
    useStopByException = True

    __optionalDataThatCanBeHandled__ = []
    __isIterPointAlwaysFeasible__ = lambda self, p: p.isUC#TODO: provide possibility of simple True, False
    iterfcnConnected = False
    funcForIterFcnConnection = 'df' # the field is used for non-linear solvers with not-connected iter function
    _canHandleScipySparse = False # True if can handle linear constraints Ax<=b, Aeq x = beq and nonlin cons derivs
    properTextOutput = False
    useLinePoints = False

    # these ones below are used in iterfcn (ooIter.py)
    # to decode input args
    # and can be overdetermined by child class (LP, QP, network etc)
    __expectedArgs__ = ['xk',  'fk',  'rk'] #point, objFunVal, max residual
    def __decodeIterFcnArgs__(self,  p,  *args,  **kwargs):
        """
        decode and assign x, f, maxConstr
        (and/or other fields) to p.iterValues
        """
        fArg  = True

        if len(args)>0 and isinstance(args[0], Point):
            if len(args) != 1: p.err('incorrect iterfcn args, if you see this contact OO developers')
            point = args[0]
            p.xk, p.fk = point.x, point.f()
            p.rk, p.rtk, p.rik = point.mr(True)
            p.nNaNs = point.nNaNs()
            if p.solver._requiresBestPointDetection and (p.iter == 0 or point.betterThan(p._bestPoint)): p._bestPoint = point
        else:
            if len(args)>0: p.xk = args[0]
            elif 'xk' in kwargs.keys(): p.xk = kwargs['xk']
            elif not hasattr(p, 'xk'): p.err('iterfcn must get x value, if you see it inform oo developers')
            if p._baseClassName == 'NonLin': 
                C = p.c(p.xk)
                H = p.h(p.xk)
                p.nNaNs = len(where(isnan(C))[0]) + len(where(isnan(H))[0])
            if p.solver._requiresBestPointDetection:
                currPoint = p.point(p.xk)
                if p.iter == 0 or currPoint.betterThan(p._bestPoint): p._bestPoint = currPoint
            if len(args)>1: p.fk = args[1]
            elif 'fk' in kwargs.keys(): p.fk = kwargs['fk']
            else: fArg = False

            if len(args)>2: 
                #p.pWarn('executing deprecated code, inform developers')
                p.rk = args[2]
            elif 'rk' in kwargs.keys(): 
                #p.pWarn('executing deprecated code, inform developers')
                p.rk = kwargs['rk']
            else:
                p.rk, p.rtk, p.rik = p.getMaxResidual(p.xk, True)
            
        
        p.iterValues.r.append(p.rk)
        if p.probType != 'IP':
            # recalculations are not performed
            p.rk, p.rtk, p.rik = p.getMaxResidual(p.xk, True)
            p.iterValues.rt.append(p.rtk)
            p.iterValues.ri.append(p.rik)
        if p._baseClassName == 'NonLin': p.iterValues.nNaNs.append(p.nNaNs)

        #TODO: handle kwargs correctly! (decodeIterFcnArgs)

#        for key in kwargs.keys():
#            if p.debug: print 'decodeIterFcnArgs>>',  key,  kwargs[key]
#            setattr(p, key, kwargs[key])

        p.iterValues.x.append(copy(p.xk))
        if not p.storeIterPoints and len(p.iterValues.x) > 2:
            p.iterValues.x.pop(0)
        
        if not fArg:
            p.Fk = p.F(p.xk)
            p.fk = copy(p.Fk)
        else:
            if asarray(p.fk).size >1:
                if p.debug and p.iter <= 1: p.warn('please fix solver iter output func, objFuncVal should be single number (use p.F)')
                p.Fk = p.objFuncMultiple2Single(asarray(p.fk))
            else:
                p.Fk = p.fk

        #if p.isObjFunValueASingleNumber: p.Fk = p.fk
        #else: p.Fk = p.objFuncMultiple2Single(fv)

        v = ravel(p.Fk)[0]
        if p.invertObjFunc: v = -v

        p.iterValues.f.append(v)
        
        if not isscalar(p.fk) and p.fk.size == 1:
            p.fk = asscalar(p.fk)
            
@vectorize([float64(float64,float64,float64)],target='cuda')
def LCPSolve(M,q, pivtol=1e-8): # pivtol = smallest allowable pivot element
        rayTerm = False
        loopcount = 0
        if (q >= 0.).all(): # Test missing in Rob Dittmar's code
        # As w - Mz = q, if q >= 0 then w = q and z = 0
            w = q
            z = np.zeros_like(q)
            retcode = 0.
        else:
            dimen = M.shape[0] # number of rows
            # Create initial tableau
            tableau = hstack([eye(dimen), -M, -ones((dimen, 1)), asarray(asmatrix(q).T)])
            # Let artificial variable enter the basis
            basis = list(range(dimen)) # basis contains a set of COLUMN indices in the tableau 
            locat = argmin(tableau[:,2*dimen+1]) # row of minimum element in column 2*dimen+1 (last of tableau)
            basis[locat] = 2*dimen # replace that choice with the row 
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
                locat = argmin(quots)
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
            vars = zeros(2*dimen+1)
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

class lcpsolve(baseSolver):
    __name__ = 'lcp'
    __license__ = "MIT"
    __authors__ = "Rob Dittmar, Enzo Michelangeli and IT Vision Ltd"
    __alg__ = "Lemke's Complementary Pivot algorithm"
    __optionalDataThatCanBeHandled__ = []
    #iterfcnConnected = True
    #_canHandleScipySparse = True
    __info__ = '''  '''
    pivtol = 1e-8

    def __init__(self): pass
    def __solver__(self, p):
        w, z, retcode = LCPSolve(p.M,p.q, pivtol=self.pivtol)
        p.xf = hstack((w, z))
        if retcode[0] == 1:
            p.istop = 1000
            p.msg = 'success'
        elif retcode[0] == 2:
            p.istop = -1000
            p.msg = 'ray termination'


particles=int(100)
size=int(particles*particles-np.ceil(particles/2))
M=np.random.rand(size,size,dtype=float32)
q=np.random.rand(size,dtype=float32)

w,z,retcode=LCPSolve(M,q)
stop = timeit.default_timer()

print(z)
print(w)
print(retcode)
print('Time: ', stop - start) 