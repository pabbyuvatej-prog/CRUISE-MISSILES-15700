import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/"src"))
import numpy as np
from av_impacttetur.models import geometry,inertia,rigid_body_angular_acceleration,run_case

def test_geometry():
    g=geometry()
    assert abs(g["L"]-0.30)<1e-12
    assert abs(g["Aref"]-np.pi*0.06**2)<1e-12

def test_inertia_symmetric():
    I=inertia()
    assert np.max(np.abs(I-I.T))<1e-15
    assert np.min(np.linalg.eigvalsh(I))>0

def test_quaternion_and_run():
    rows=run_case(duration=0.01)
    assert np.max(np.abs(rows[:,15]-1))<1e-10
