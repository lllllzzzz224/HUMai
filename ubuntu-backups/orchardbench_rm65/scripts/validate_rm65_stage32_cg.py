"""Original 40-test suite with CG constructor selection; no source changes."""
from pathlib import Path
import sys,unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import newton
real=newton.solvers.SolverMuJoCo
count=0
def cg(*args,**kwargs):
    global count
    kwargs['solver']=1
    solver=real(*args,**kwargs)
    assert int(solver.mj_model.opt.solver)==int(solver.mjw_model.opt.solver)==1
    count+=1
    return solver
with patch.object(newton.solvers,'SolverMuJoCo',side_effect=cg):
    suite=unittest.defaultTestLoader.loadTestsFromNames(['tests.test_stage32','tests.test_stage32_c','tests.test_stage31b'])
    assert suite.countTestCases()==40
    result=unittest.TextTestRunner(verbosity=2,failfast=True).run(suite)
    if not result.wasSuccessful():raise SystemExit(1)
print(f'CG runtime constructor assertions passed: {count}')
