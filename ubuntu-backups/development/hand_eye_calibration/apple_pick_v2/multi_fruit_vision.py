"""Complete snapshot adapter, shared by live perception and deterministic replay."""
import time
from multi_fruit_state import deduplicate


def covered_partial(partial, complete):
    """An edge fragment is usable only when a complete box covers most of it."""
    if 'global_xyxy' not in partial or 'global_xyxy' not in complete:
        return False
    if partial.get('class_id') != complete.get('class_id'):
        return False
    a, b = partial['global_xyxy'], complete['global_xyxy']
    overlap = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
    area = max(1., (a[2]-a[0])*(a[3]-a[1]))
    return overlap/area >= .8


def build_snapshot(capture_stamp_s, candidates, estimate, *, frame='base_link', model_duration_s=None):
    complete = [c for c in candidates if not c['touches_edge']]
    valid = all(any(covered_partial(c, full) for full in complete)
                for c in candidates if c['touches_edge'])
    unique = []
    for candidate in sorted(complete,key=lambda c:-c.get('confidence',0.)):
        if 'global_xyxy' in candidate and any(
                covered_partial(candidate, prior) and covered_partial(prior, candidate)
                for prior in unique):
            continue
        unique.append(candidate)
    started = time.perf_counter()
    detections = []
    for candidate in unique:
        item = estimate(candidate)
        if item is None:
            valid = False
        else:
            detections.append(item)
    return dict(version=1, frame=frame, capture_stamp_s=float(capture_stamp_s),
                valid=valid, detections=deduplicate(detections),
                source_model_duration_s=model_duration_s,
                source_localization_duration_s=time.perf_counter()-started,
                reason='complete' if valid else 'partial_or_unlocalized_detection')
