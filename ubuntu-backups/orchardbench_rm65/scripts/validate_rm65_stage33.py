"""Task-scale closure; strict Stage 3.3 validator remains unchanged."""
import sys, json, argparse
from datetime import datetime
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scripts import grow_tree
from scripts.validate_rm65_kinematics import ValidationProgram, state_pose
from treesim.rm65_kinematics import pose_error, target_pose_base_to_world
from treesim.rm65_planning import PlanningFailure
from treesim.rm65_runtime import emit
from treesim import rm65_control as rc
from treesim.rm65_acceptance import EnvironmentClearance as Clearance, ArrivalWindow, repeatability

class TaskScaleProgram(ValidationProgram):

    def __init__(self, output):
        super().__init__()
        self.output = Path(output)
        self.rows = None
        self.gate = ArrivalWindow()

    def save(self, name, data):
        with (self.output / name).open('x') as f:
            json.dump(data, f, indent=2, default=lambda a: a.tolist())

    def prepare(self, r):
        super().prepare(r)
        if len(self.plan) != 129:
            raise PlanningFailure('PLAN_SIZE', len(self.plan))
        self.clearance = Clearance(r)
        before = self.live_snapshot()
        q, _ = r.controller.read_arm_state(r.sim.state_0)
        for w in self.plan:
            while True:
                self.scratch.check(q)
                self.clearance.check(self.scratch.state, 'preflight/' + w.label)
                if np.array_equal(q, w.q_arm):
                    break
                q += np.clip(w.q_arm - q, -rc.RATE[:6] / 60, rc.RATE[:6] / 60)
        self.assert_live_snapshot(before)
        assert r.substeps == 0
        self.save('execution-preflight.json', dict(minimum=self.clearance.minimum, installation_min_m=self.clearance.install_min, samples=self.clearance.samples, physics_steps=0, ik_source='fresh strict ValidationProgram.prepare', plan=[dict(label=w.label, q_arm=w.q_arm, target_pose_base=w.target_pose_base) for w in self.plan], whole_three_rounds_checked=True))
        self.rows = (self.output / 'task-scale-substeps.jsonl').open('x')
        self.endpoints = []
        self.stage_elapsed = 0
        self.stage_speed = 0.0
        self.stage_min = None
        self.stage_key = None
        self.motion_min = None
        emit(event='task_scale_prepared', plan_size=len(self.plan), minimum=self.clearance.minimum)

    def stage(self):
        w = self.plan[self.index]
        parts = w.label.split('/')
        leaf = parts[1]
        if leaf == 'approach':
            leaf = 'grasp'
        if leaf == 'home' and self.index % 43 == 0:
            leaf = 'initial_home'
        return (parts[0] + '/' + leaf, leaf)

    def before_frame(self, r):
        key, stage = self.stage()
        if key != self.stage_key:
            self.stage_key = key
            self.stage_elapsed = 0
            self.stage_speed = 0.0
            self.stage_min = None
        if not self.active:
            self.gate.reset()
        super().before_frame(r)

    def after_substep(self, state):
        r = self.runtime
        c = r.controller
        q, v = c.read_arm_state(state)
        if np.any(q < c.limits['lower'][:6]) or np.any(q > c.limits['upper'][:6]):
            raise PlanningFailure('HARD_LIMIT', q)
        row, installation = self.clearance.check(state, 'execution/' + self.plan[self.index].label)
        if self.stage_min is None or row['distance_m'] < self.stage_min['distance_m']:
            self.stage_min = row
        if self.motion_min is None or row['distance_m'] < self.motion_min['distance_m']:
            self.motion_min = row
        actual = state_pose(state.body_q.numpy()[r.audit.body_ids['gripper_tcp']])
        fk = target_pose_base_to_world(self.kin.fk(q), self.T_world_base)
        fp, fr = pose_error(actual, fk)
        self.max_fk_position = max(self.max_fk_position, fp)
        self.max_fk_rotation = max(self.max_fk_rotation, fr)
        if fp > 0.0001 or fr > np.deg2rad(0.01):
            raise PlanningFailure('ACTUAL_FK_MISMATCH', dict(position=fp, rotation=fr))
        target = target_pose_base_to_world(self.plan[self.index].target_pose_base, self.T_world_base)
        ep, er = pose_error(actual, target)
        self.elapsed += 1
        self.stage_elapsed += 1
        self.stage_speed = max(self.stage_speed, float(np.max(np.abs(v))))
        self.gate.update(ep, er, v)
        self.consecutive = self.gate.count
        self.last = dict(label=self.plan[self.index].label, stage=self.stage()[1], actual_q=q, actual_qd=v, joint_error=c.goal[:6] - q, target=c.goal[:6].copy(), effective_target=c.sent[:6].copy(), actual_pose_world=actual, target_pose_world=target, position_error=ep, rotation_error=er, consecutive=self.consecutive, segment_time_s=self.stage_elapsed / 180, waypoint_time_s=self.elapsed / 180, segment_max_speed=self.stage_speed, environment_min=self.stage_min, installation_distance_m=installation, actual_fk_position_error=fp, actual_fk_rotation_error=fr, physics_step=r.substeps)
        self.rows.write(json.dumps(self.last, default=lambda a: a.tolist()) + '\n')
        if r.substeps % 180 == 0:
            self.rows.flush()
        is_endpoint = self.index % 43 in (0, 1, 21, 41, 42)
        if self.stage_elapsed >= 1800 and (not (self.consecutive >= 45 and is_endpoint)):
            emit(event='task_scale_timeout', **self.last)
            raise PlanningFailure('TIMEOUT', self.last)

    def after_frame(self, r):
        if self.consecutive < 45:
            return
        self.completed.append(self.plan[self.index].label)
        if self.index % 43 in (0, 1, 21, 41, 42):
            endpoint = json.loads(json.dumps(self.last, default=lambda a: a.tolist()))
            self.endpoints.append(endpoint)
            emit(event='task_scale_endpoint', **endpoint)
        self.index += 1
        self.active = False
        if self.index == len(self.plan):
            self.rows.close()
            self.save('endpoints.json', self.endpoints)
            rep = repeatability(self.endpoints)
            self.save('sequence-result.json', dict(result='PASS', rounds=3, waypoints_completed=len(self.completed), endpoints=self.endpoints, repeatability=rep, execution_minimum=self.motion_min, installation_min_m=self.clearance.install_min, installation_max_m=self.clearance.install_max, actual_fk_max_position=self.max_fk_position, actual_fk_max_rotation=self.max_fk_rotation, frames=r.frames, substeps=r.substeps))
            self.done = True
            emit(event='task_scale_sequence', result='PASS', rounds=3, repeatability=rep)

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=None, help='New directory for generated evidence; never used as input')
    args = parser.parse_args(argv)
    output = args.log_dir or Path('output') / ('stage33-acceptance-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    output.mkdir(parents=True, exist_ok=False)
    p = TaskScaleProgram(output)
    try:
        grow_tree.main(['--robot-model', 'rm65', '--mount', 'fixed', '--rm-control', 'joint', '--viewer', 'null', '--frames', '30000', '--substeps', '3', '--seed', '31'], rm_program=p)
    except Exception as exc:
        p.save('sequence-stop.json', dict(category=getattr(exc, 'category', type(exc).__name__), error=str(exc), completed=p.completed, endpoints=getattr(p, 'endpoints', []), last=getattr(p, 'last', None)))
        raise
    finally:
        if p.rows is not None and (not p.rows.closed):
            p.rows.close()
if __name__ == '__main__':
    main()
