from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent/"src"))
from av_impacttetur.models import run_case, save_csv, atmosphere_from_reference, geometry, thermal_state, orthotropic_surrogate

def main():
    out=Path("outputs"); out.mkdir(exist_ok=True)
    rows=run_case()
    save_csv(out/"baseline_6dof_results.csv",rows)
    print("AV Impacttetur simulation")
    print("==========================")
    print(f"Steps: {len(rows)}")
    print(f"Final altitude: {rows[-1,1]:.6f} m")
    print(f"Final velocity: {rows[-1,2]:.6f} m/s")
    print(f"Final alpha:    {rows[-1,3]:.6f} rad")
    print(f"Final q-rate:   {rows[-1,5]:.6f} rad/s")
    print(f"Final delta:    {rows[-1,6]:.6f} rad")
    print(f"Quaternion norm:{rows[-1,15]:.12f}")
    print("CSV written to outputs/baseline_6dof_results.csv")
    print("NOTE: this is a reduced numerical research model; physical validation is not implied.")

if __name__=="__main__":
    main()
