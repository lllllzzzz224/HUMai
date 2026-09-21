"""Independent RM official_assist entry using the accepted shared fixture."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from treesim.rm65_run_entry import main as run_entry

def main(argv=None):
    return run_entry(argv,revalidate_only=False)

if __name__=='__main__':raise SystemExit(main())
