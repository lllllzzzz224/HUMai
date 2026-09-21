"""Single immutable seed35 fixture source; no scoring payload reaches control."""
from dataclasses import dataclass, asdict
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

DEFAULT_FIXTURE = Path(__file__).resolve().parents[1] / 'output/stage35-pick-center15-20260921/locked-target-and-fixture.json'
# User-approved center-only revision; the original REVALIDATE fixture is preserved.
ACCEPTED_SHA256 = '874d2e1601f8a1e3c99925819e82a0d052c1c9f308bf90dcb32dba00e64169d1'

@dataclass(frozen=True)
class RMRunConfig:
    seed: int
    apple: str
    stem: str
    mount: tuple
    tcp_quaternion_xyzw: tuple
    pregrasp_offset_tool: tuple
    bucket_floor: tuple
    bucket_yaw: float
    fixture_sha256: str
    source_path: str
    grasp_center_local: tuple
    mode: str = 'official_assist'

    def record(self):
        return asdict(self)

def load_config(path=DEFAULT_FIXTURE):
    if path is None:
        raise ValueError('MISSING_FIXTURE')
    path = Path(path).resolve()
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != ACCEPTED_SHA256:
        raise ValueError('FIXTURE_HASH_MISMATCH')
    data = json.loads(raw)
    center = np.asarray(data['grasp_center_local'], dtype=float)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError('INVALID_GRASP_CENTER')
    rotation = np.asarray(data['approach_rotation'], dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all() or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(rotation), 1.):
        raise ValueError('INVALID_NOMINAL_TCP_ROTATION')
    if data['bucket_yaw'] != 0.:
        raise ValueError('UNSUPPORTED_BUCKET_YAW')
    return RMRunConfig(seed=data['seed'], apple=data['apple'], stem=data['stem'],
        mount=tuple(data['mount']), tcp_quaternion_xyzw=tuple(Rotation.from_matrix(rotation).as_quat()),
        pregrasp_offset_tool=tuple(data['pregrasp_offset_tool']), bucket_floor=tuple(data['bucket_floor']),
        bucket_yaw=data['bucket_yaw'], fixture_sha256=digest, source_path=str(path),
        grasp_center_local=tuple(center))

def scene_args(config):
    return ['--robot-model','rm65','--mount','fixed','--rm-control','joint','--rm-camera',
        '--apples','--viewer','gl','--seed',str(config.seed),'--frames','20000',
        '--stage35-fixed-mount',*map(str,config.mount)]
