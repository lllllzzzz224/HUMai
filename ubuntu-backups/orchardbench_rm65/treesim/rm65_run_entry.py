"""Shared fixture loading and scene construction for both RM entrypoints."""
import argparse
import json
import os
from pathlib import Path
import traceback
from contextlib import nullcontext
from .rm65_run_config import DEFAULT_FIXTURE, load_config, scene_args
from .rm65_official import Bucket, bucket_scene
from .rm65_official_strategy import LocalGraspStrategy
from .rm65_fixture_diagnostic import FixtureRevalidateOnly

def main(argv=None, *, revalidate_only):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fixture',type=Path,default=DEFAULT_FIXTURE)
    p.add_argument('--log-dir',required=True,type=Path)
    p.add_argument('--config-only',action='store_true',help='Validate wiring only; no simulation, GUI, physics or commands')
    p.add_argument('--pick-only',action='store_true',help='No bucket or release; stop after verified detach and safe hold')
    a=p.parse_args(argv)
    if a.pick_only and revalidate_only:p.error('--pick-only is only available in the official_assist entry')
    config=load_config(a.fixture)  # fail before any output/scene construction
    from scripts import grow_tree
    args=scene_args(config)
    scene=grow_tree.make_config(grow_tree.parse_args(args))
    bucket=None if a.pick_only else Bucket(config.bucket_floor,yaw=config.bucket_yaw)
    out=a.log_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    work=out/'viewer-workdir';work.mkdir();original=Path.cwd()
    if a.pick_only:
        from .rm65_pick_only import PickOnlyStrategy
        program=PickOnlyStrategy(out,config=config)
    else:
        program=(FixtureRevalidateOnly(out,bucket,config) if revalidate_only else
                 LocalGraspStrategy(out,bucket,config=config))
    program.save('configuration.json',dict(runtime_config=config.record(),
        fixture_sha256=config.fixture_sha256,pid=os.getpid(),argv=args,
        revalidate_only=revalidate_only,config_only=a.config_only,task_mode='pick_only' if a.pick_only else 'full'))
    error=None
    try:
        if a.config_only:
            # Exercise the same adapter factory as prepare(), without building a scene.
            from types import SimpleNamespace
            resolved=program.create_gripper(SimpleNamespace(audit=SimpleNamespace(body_ids={'gripper_tcp':-1})))
            record=dict(status='CONFIG_PREFLIGHT_PASS',fixture_sha256=config.fixture_sha256,
                grasp_center_local=resolved.grasp_center_local,resolved_hold_anchor=resolved.offset,
                scene_seed=scene.seed,strategy_target=program.config.apple,strategy_stem=program.config.stem,
                strategy_mode=program.config.mode,mount=scene.robot.stage35_fixed_mount,
                nominal_tcp_quaternion_xyzw=config.tcp_quaternion_xyzw,
                approach_policy='retain measured stable READY orientation; translate along configured tool offset',
                pregrasp_offset_tool=program.offset,bucket_floor=None if bucket is None else bucket.floor,bucket_yaw=None if bucket is None else bucket.yaw,
                task_mode='pick_only' if a.pick_only else 'full',
                physics_steps=0,viewer_created=False,scene_built=False,motion_commands=0)
            program.save('config-preflight.json',record)
            print(json.dumps(record,default=lambda x:x.tolist() if hasattr(x,'tolist') else x))
            return 0
        os.chdir(work)
        with (nullcontext() if a.pick_only else bucket_scene(bucket)):
            grow_tree.main(args,rm_program=program)
        if not program.done:raise RuntimeError('INCOMPLETE')
    except BaseException as exc:
        error=f'{type(exc).__name__}: {exc}'
        program.save('traceback.json',dict(traceback=traceback.format_exc()))
        try:program.fail(exc)
        except BaseException as secondary:program.save('failure-saving-error.json',dict(error=str(secondary)))
    finally:
        if revalidate_only and not a.config_only:program.diagnostic_result(error)
        program.close_files();os.chdir(original)
    expected='REVALIDATE_ONLY_DONE' if revalidate_only else ('PICK_HOLD_DONE' if a.pick_only else 'DONE')
    return 0 if not error and not program.failure and program.done and program.state==expected else 1
