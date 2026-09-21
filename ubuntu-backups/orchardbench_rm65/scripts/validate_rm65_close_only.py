"""One native-GUI Stage 3.5 close_only run; no automatic retry."""
import argparse
import json
import os
from pathlib import Path
import sys
import traceback
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts import grow_tree
from treesim.rm65_close_only import CloseOnlyProgram, MOUNT, READY


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir',type=Path,required=True)
    parser.add_argument('--pregrasp-only',action='store_true',
        help='Stage 3.5 opt-in 5 mm warning / 8 mm free-space hard gate; stop at stable PREGRASP')
    args=parser.parse_args(argv)
    out=args.log_dir.resolve(); out.mkdir(parents=True,exist_ok=False)
    work=out/'viewer-workdir'; work.mkdir()
    program=CloseOnlyProgram(out,free_space_tracking=args.pregrasp_only,pregrasp_only=args.pregrasp_only)
    program.save('configuration.json',dict(mode='pregrasp_only' if args.pregrasp_only else 'close_only',
        free_space_tracking_policy=args.pregrasp_only,seed=31,apple='apple35',parent='seg294',
        mount=MOUNT,ready=READY,pid=os.getpid(),native_gui=True,hold=False,detach=False,retract=False))
    original=Path.cwd()
    try:
        os.chdir(work)
        grow_tree.main(['--robot-model','rm65','--mount','fixed','--rm-control','joint',
            '--rm-camera','--apples','--viewer','gl','--seed','31','--frames','15000',
            '--stage35-fixed-mount',*[str(x) for x in MOUNT]],rm_program=program)
        expected='PREGRASP_ONLY_DONE' if args.pregrasp_only else 'CLOSE_ONLY_DONE'
        if program.state!=expected: raise RuntimeError('INCOMPLETE_'+expected)
    except BaseException as exc:
        program.fail(exc)
        program.save('traceback.json',dict(traceback=traceback.format_exc()))
        raise
    finally:
        program.close_files(); os.chdir(original)


if __name__=='__main__': main()
