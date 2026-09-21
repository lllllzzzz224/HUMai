"""Run one fixed-seed diagnostic, never beyond fresh PREGRASP revalidation."""
import argparse
import os
import sys
import traceback
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts import grow_tree
from treesim.rm65_official import Bucket,bucket_scene,MOUNT
from treesim.rm65_seed_diagnostic import RevalidateOnly

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seed',type=int,choices=(32,33,34,35,36),required=True)
    p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--expected-target')
    a=p.parse_args(argv)
    out=a.log_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    work=out/'viewer-workdir';work.mkdir();original=Path.cwd()
    bucket=Bucket((.8,.7,0.));program=RevalidateOnly(out,bucket,a.expected_target)
    args=['--robot-model','rm65','--mount','fixed','--rm-control','joint','--rm-camera',
        '--apples','--viewer','gl','--seed',str(a.seed),'--frames','20000','--stage35-fixed-mount',*map(str,MOUNT)]
    program.save('configuration.json',dict(seed=a.seed,expected_target=a.expected_target,mount=MOUNT,
        bucket_floor=bucket.floor,mode='official_assist/revalidate_only',pid=os.getpid(),argv=args,
        inherited_pregrasp_offset_tool_m=[.02,0.,-.03]))
    error=None
    try:
        os.chdir(work)
        with bucket_scene(bucket):grow_tree.main(args,rm_program=program)
        if not program.done:raise RuntimeError('INCOMPLETE_DIAGNOSTIC')
    except BaseException as exc:
        error=f'{type(exc).__name__}: {exc}'
        program.save('traceback.json',dict(traceback=traceback.format_exc()))
        try:program.fail(exc)
        except BaseException as secondary:program.save('failure-saving-error.json',dict(error=str(secondary)))
    finally:
        result=program.diagnostic_result(error);program.close_files();os.chdir(original)
    return 0 if result['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())
