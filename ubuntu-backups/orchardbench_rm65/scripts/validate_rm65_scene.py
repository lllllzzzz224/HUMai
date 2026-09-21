"""G: validate the formal grow_tree RM joint-control entry, without loader patches."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts import grow_tree
from treesim.rm65_runtime import emit


def main():
    args = grow_tree.parse_args()
    if (args.robot_model,args.mount,args.rm_control,args.viewer,args.substeps,args.num_envs) != ('rm65','fixed','joint','null',3,1):
        raise ValueError('G requires rm65+fixed, --rm-control joint, null viewer, 3 substeps, one environment')
    if args.frames <= 0 or args.seed < 0:
        raise ValueError('G requires finite frames and a fixed seed')
    runtime = grow_tree.main()
    if runtime.frames != args.frames or runtime.max_error > .001 or runtime.max_speed > .15:
        raise RuntimeError('SAFE_PERFORMANCE_FAILURE: orchard home hold')
    emit(stage='G', result='PASS', **runtime.snapshot())


if __name__ == '__main__':
    main()
