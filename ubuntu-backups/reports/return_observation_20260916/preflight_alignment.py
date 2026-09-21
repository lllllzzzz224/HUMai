from pathlib import Path
import sys
sys.path.insert(1,'/home/li/桌面/9月机械臂实验（算法）/reports/vertical_diagnostic20_20260916')
from importlib.util import spec_from_file_location,module_from_spec
spec=spec_from_file_location('readonly_parent','/home/li/桌面/9月机械臂实验（算法）/reports/vertical_diagnostic20_20260916/preflight_alignment.py');m=module_from_spec(spec);spec.loader.exec_module(m)
ReadOnly=m.ReadOnly;load_config=m.load_config;P=Path(__file__).resolve().parent
