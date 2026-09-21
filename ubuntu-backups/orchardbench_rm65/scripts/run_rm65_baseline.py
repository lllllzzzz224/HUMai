"""Run the fixed RM author-baseline example with a CPU display of Newton frames.

The original SensorTiledCamera still supplies perception. This desktop mirror only
avoids the local GL presentation black-screen issue; it never feeds the detector.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def display(directory):
    import tkinter as tk
    from PIL import Image, ImageTk
    root = tk.Tk()
    root.title('RM65 author baseline - truth-assisted / spring-assisted')
    root.geometry('1100x810+80+70')
    label = tk.Label(root, text='Preparing RM scene and collision checks...', bg='white', fg='black')
    label.pack(fill='x')
    panel = tk.Label(root, bg='white')
    panel.pack(fill='both', expand=True)
    stamp = None

    def refresh():
        nonlocal stamp
        try:
            path = directory/'frame.png'
            if path.exists() and path.stat().st_mtime_ns != stamp:
                im = Image.open(path)
                im.thumbnail((1080, 750))
                photo = ImageTk.PhotoImage(im)
                panel.configure(image=photo)
                panel.image = photo
                stamp = path.stat().st_mtime_ns
            status = directory/'display-status.json'
            if status.exists():
                label.configure(text=json.loads(status.read_text())['message'])
        except (OSError, ValueError):
            pass
        root.after(100, refresh)
    root.after(50, refresh)
    root.lift()
    root.attributes('-topmost', True)
    root.after(1500, lambda: root.attributes('-topmost', False))
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--display', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--frames', type=int, default=3600)
    parser.add_argument('--log-dir', type=Path)
    args = parser.parse_args()
    if args.display:
        display(args.display)
        return
    if args.frames <= 0:
        parser.error('--frames must be positive')
    directory = (args.log_dir or ROOT/'output'/('rm-baseline-'+datetime.now().strftime('%Y%m%d-%H%M%S'))).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    work = directory/'viewer-workdir'
    work.mkdir()
    os.chdir(work)
    child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--display', str(directory)])
    from PIL import Image
    from scripts import grow_tree
    import warp as wp
    import numpy as np
    factory = grow_tree.make_viewer
    last_capture = 0.

    def status(message):
        tmp = directory/'display-status.tmp'
        tmp.write_text(json.dumps(dict(message=message)))
        os.replace(tmp, directory/'display-status.json')

    def viewer_factory(a):
        viewer = factory(a)
        viewer.renderer.window.set_visible(False)
        original_end, original_running = viewer.end_frame, viewer.is_running

        def end_frame():
            nonlocal last_capture
            original_end()
            if time.monotonic()-last_capture > .1:
                Image.fromarray(viewer.get_frame().numpy()).save(directory/'frame.tmp.png')
                os.replace(directory/'frame.tmp.png', directory/'frame.png')
                last_capture = time.monotonic()
        viewer.end_frame = end_frame
        viewer.is_running = lambda: original_running() and child.poll() is None
        return viewer
    grow_tree.make_viewer = viewer_factory
    from treesim import rm65_picker
    original_picker = rm65_picker.RMBaselinePicker
    active = []

    class DisplayPicker(original_picker):
        def __init__(self, *values):
            super().__init__(*values)
            active.append(self)
            v = self.sim.viewer
            pos = np.array([2.45, -1.55, 1.55]); look = np.array([.87, .36, .65])
            d = look-pos; d /= np.linalg.norm(d)
            v.set_camera(wp.vec3(*pos), pitch=float(np.degrees(np.arcsin(d[2]))),
                         yaw=float(np.degrees(np.arctan2(d[1], d[0]))))

        def update(self):
            super().update()
            status(f'{self.state} | frame {self._frame} | truth-assisted baseline | spring-assisted hold')
    rm65_picker.RMBaselinePicker = DisplayPicker
    command = ['--robot-model','rm65','--mount','fixed','--rm-control','joint',
        '--rm-auto-baseline','--seed','31','--viewer','gl','--frames',str(args.frames),
        '--stage35-fixed-mount','1.219252813007449','0.347841569121514','0','2.8797932657906435',
        '--rm-ready','-0.4461756944656372','1.2990716695785522','-0.9880963563919067',
        '0.2896624803543091','-1.523876428604126','0.09278970211744308']
    (directory/'command.json').write_text(json.dumps(command, indent=2))
    try:
        runtime = grow_tree.main(command)
        picker = active[0]
        result = dict(state=picker.state, done=picker.done, physics_steps=runtime.substeps,
                      target='apple35', truth_assisted=True, spring_assisted_hold=True,
                      detached=bool(picker.apples.detached[picker.target_index]),
                      held=bool(picker.apples._held_host[picker.target_index]))
        (directory/'result.json').write_text(json.dumps(result, indent=2))
        status('STOPPED: '+json.dumps(result))
        print(json.dumps(result))
    except Exception as exc:
        result = dict(result='FAIL', error=str(exc), state=active[0].state if active else 'INITIALIZATION')
        (directory/'stop.json').write_text(json.dumps(result, indent=2))
        status('STOPPED: '+str(exc))
        raise
    # The CPU window deliberately remains open on the last image for inspection.


if __name__ == '__main__':
    main()
