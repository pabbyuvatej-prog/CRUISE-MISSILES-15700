from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parents[1]/"src"))
from av_impacttetur.models import run_case, save_csv
rows=run_case(duration=0.10, dt=0.0005)
save_csv(Path(__file__).parents[1]/"outputs/example_baseline.csv", rows)
print("Example complete.")
