"""Pure, conservative spatial scheduling. Track IDs are hypotheses, not fruit identity."""
from dataclasses import dataclass, replace
from math import dist, isfinite


def deduplicate(detections, tolerance_m=.012):
    result = []
    for item in sorted(detections, key=lambda d: -d['confidence']):
        if not any(item['class_id'] == other['class_id'] and
                   dist(item['center_xyz_m'], other['center_xyz_m']) < tolerance_m
                   for other in result):
            result.append(item)
    return result


def validate_snapshot(value, frame='base_link'):
    if (value.get('version') != 1 or value.get('frame') != frame or
            type(value.get('valid')) is not bool):
        raise ValueError('snapshot version/frame/valid invalid')
    stamp = float(value['capture_stamp_s'])
    if not isfinite(stamp) or stamp <= 0:
        raise ValueError('invalid source capture stamp')
    detections = value['detections']
    if not isinstance(detections, list):
        raise ValueError('detections must be a complete list')
    for d in detections:
        xyz = d['center_xyz_m']
        if (len(xyz) != 3 or not all(isfinite(float(v)) for v in xyz) or
                type(d['class_id']) is not int or not isinstance(d['class_name'], str) or
                not 0 <= float(d['confidence']) <= 1 or
                not 0 < float(d['radius_m']) < .2):
            raise ValueError('invalid detection')
    return stamp, deduplicate(detections)


@dataclass
class Track:
    id: str
    class_id: int
    class_name: str
    confidence: float
    center: tuple
    radius_m: float
    capture: float
    stable: int = 1
    attempted: bool = False
    ambiguous: bool = False
    blocked_until: float = 0.
    attempts: int = 0


class TargetManager:
    def __init__(self, class_id=49, frame='base_link', stable_frames=2,
                 max_age_s=1., association_m=.045, stable_m=.008,
                 min_confidence=.15, min_radius_m=.02, max_radius_m=.038,
                 empty_frames=2, empty_interval_s=.05):
        self.class_id, self.frame = class_id, frame
        self.stable_frames, self.max_age_s = stable_frames, max_age_s
        self.association_m, self.stable_m = association_m, stable_m
        self.min_confidence = min_confidence
        self.min_radius_m, self.max_radius_m = min_radius_m, max_radius_m
        self.tracks = {}
        self.visible = set()
        self.capture = 0.
        self.boundary = 0.
        self.valid = False
        self.locked = None
        self.scene_revision = 0
        self.empty_frames, self.empty_interval_s = max(2,empty_frames), empty_interval_s
        self.empty_count, self.empty_since = 0, None

    def update(self, value, now):
        try:
            stamp, detections = validate_snapshot(value, self.frame)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.valid = False
            self.empty_count, self.empty_since = 0, None
            self.scene_revision += 1
            return False
        if stamp <= self.capture:
            return False
        self.capture = stamp
        self.valid = value['valid'] and 0 <= now-stamp <= self.max_age_s
        if not self.valid:
            self.visible = set()
            self.empty_count, self.empty_since = 0, None
            self.scene_revision += 1
            return False
        # Other classes are retained as obstacles by the ROS adapter; scheduling is same class.
        detections = [d for d in detections if d['class_id'] == self.class_id]
        if detections:
            self.empty_count, self.empty_since = 0, None
        else:
            self.empty_count += 1
            if self.empty_since is None: self.empty_since = stamp
        candidates = [[t for t in self.tracks.values()
                       if t.class_id == d['class_id'] and dist(t.center,d['center_xyz_m']) <= self.association_m]
                      for d in detections]
        uses = {t.id: sum(t in group for group in candidates) for t in self.tracks.values()}
        missing_prior = any(uses[p.id] == 0 for p in self.tracks.values())
        visible = set()
        scene_changed = False
        for d, group in zip(detections,candidates):
            xyz = tuple(map(float,d['center_xyz_m']))
            if len(group) == 1 and uses[group[0].id] == 1:
                t = group[0]
                movement = dist(t.center,xyz)
                was_visible = t.id in self.visible
                t.stable = t.stable+1 if movement <= self.stable_m and was_visible else 1
                # A unique spatial association can restabilize; ambiguous groups stay quarantined.
                scene_changed |= movement > self.stable_m
                t.center, t.confidence, t.radius_m, t.capture = xyz, d['confidence'], d['radius_m'], stamp
            else:
                for prior in group:
                    prior.ambiguous = True
                t = Track(f'fruit-{len(self.tracks)+1}', d['class_id'], d['class_name'],
                          d['confidence'], xyz, d['radius_m'],stamp)
                # Never grant a fresh eligible ID to a displaced/missing old fruit.
                t.ambiguous = bool(group) or missing_prior
                self.tracks[t.id] = t
                scene_changed = True
            visible.add(t.id)
        if visible != self.visible or scene_changed:
            self.scene_revision += 1
        self.visible = visible
        return True

    def motion_boundary(self, capture_time):
        self.boundary = max(self.boundary, capture_time)
        for track in self.tracks.values(): track.stable = 0
        self.locked = None
        self.empty_count, self.empty_since = 0, None

    def fresh(self, now):
        return self.valid and self.capture > self.boundary and 0 <= now-self.capture <= self.max_age_s

    def eligible(self, now):
        if not self.fresh(now):
            return []
        return sorted((t for t in self.tracks.values() if t.id in self.visible and
                       not t.attempted and not t.ambiguous and t.stable >= self.stable_frames and
                       t.blocked_until <= now and t.confidence >= self.min_confidence and
                       self.min_radius_m <= t.radius_m <= self.max_radius_m),
                      key=lambda t: (-t.confidence,t.id))

    def lock(self, now):
        if self.locked and self.revalidate(self.locked,now):
            return self.locked
        eligible = self.eligible(now)
        self.locked = replace(eligible[0]) if eligible else None
        return self.locked

    def revalidate(self, frozen, now):
        return any(t.id == frozen.id and dist(t.center,frozen.center) <= self.stable_m
                   for t in self.eligible(now))

    def mark_attempted(self, identity):
        t = self.tracks[identity]
        t.attempted = True
        t.attempts += 1
        self.locked = None

    def mark_blocked(self, identity, now, cooldown_s=30., max_attempts=2):
        t = self.tracks[identity]
        t.attempts += 1
        t.blocked_until = float('inf') if t.attempts >= max_attempts else now+cooldown_s
        self.locked = None

    def summary(self, now):
        if self.capture == 0 or now-self.capture > self.max_age_s or self.capture <= self.boundary:
            return 'STALE_PERCEPTION'
        if not self.valid:
            return 'INVALID_PERCEPTION'
        if not self.visible:
            return ('SCENE_EMPTY' if self.empty_count >= self.empty_frames and
                    self.capture-self.empty_since >= self.empty_interval_s else 'NO_SAFE_TARGET')
        if self.eligible(now):
            return 'READY'
        if all(self.tracks[i].attempted for i in self.visible):
            return 'ALL_ATTEMPTED'
        return 'NO_SAFE_TARGET'
