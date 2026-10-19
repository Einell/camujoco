"""Slow algorithm consumer while the desktop continues rendering.

PYTHONPATH=src:.runtime/python-deps MUJOCO_GL=egl python3 examples/consume_frames.py
Use the same Qt system libraries as run.sh if your machine needs local xcb libs.
"""
import json
from queue import Empty
from threading import Thread
from time import sleep
from PySide6.QtWidgets import QApplication
from camera_lab.api import DataAPI
from camera_lab.app import Window


def main():
    api=DataAPI();images=api.subscribe_observations()
    result={'consumed':0,'last_batch':None}
    def consume():
        while not api.closed:
            try:batch=images.get_nowait()
            except Empty:sleep(.005);continue
            # Replace this delay with an image-only algorithm.
            result['consumed']+=1;result['last_batch']=batch.sequence
            result['shapes']=[list(f.rgb.shape) for f in batch.frames]
            sleep(.1)
    consumer=Thread(target=consume);consumer.start()
    app=QApplication([])
    window=Window(api=api,seconds=5,warmup=1)
    window.show();app.exec();consumer.join()
    result['dropped']=images.dropped
    result['produced']=window.worker.rendered
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
