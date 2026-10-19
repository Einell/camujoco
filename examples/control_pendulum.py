"""Synthetic command producer, not a robot policy; view motion in the desktop."""
import json
import math
from queue import Empty,Full
from time import monotonic_ns
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from camera_lab.api import DataAPI
from camera_lab.app import Window
from camera_lab.contracts import ControlCommand


def main():
    api=DataAPI();results=api.subscribe_control_results()
    app=QApplication([]);window=Window(api=api,seconds=5,warmup=1)
    counts={'accepted':0,'rejected':0}
    def control():
        if window.last_batch is None:return
        stamp=window.last_batch.stamp
        command=ControlCommand(stamp.run_id,stamp.model_revision,stamp.step_index+30,
                               stamp.step_index+150,monotonic_ns()+500_000_000,
                               {'swing_control':.35*math.sin(stamp.sim_time_s*3)})
        try:api.submit_control(command)
        except (Full,RuntimeError):return
        while True:
            try:result=results.get_nowait()
            except Empty:break
            counts['accepted' if result.accepted else 'rejected']+=1
    timer=QTimer();timer.timeout.connect(control);timer.start(100)
    window.show();app.exec();timer.stop()
    print(json.dumps(counts,indent=2))


if __name__=='__main__':main()
