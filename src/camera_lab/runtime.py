"""Qt-free runtime shared by the desktop and Python SDK.

MuJoCo state and GL contexts belong exclusively to Worker. SDK requests are
serialized with desktop commands and completed only after execution.
"""
from concurrent.futures import Future
from dataclasses import dataclass, replace
from numbers import Integral
from queue import Queue, Empty, SimpleQueue
from threading import Thread, Event, Lock, current_thread
from time import perf_counter
from .render.worker import RenderSession, LatestBatch
from .project.importer import SceneProject
from .project.store import save_project, load_project
from .sim.engine import Simulation, Mode
from .api.streams import DataAPI
from .metrics import Measurement


@dataclass
class _Request:
    command: str
    payload: object
    future: Future


class _DiscardEvents:
    def put(self, notice):
        pass


class Worker(Thread):
    def __init__(self, width, height, fps, model=None, packages=None, api=None, *, running=None, continuous=True, fallback=True, events=True):
        super().__init__(name='mujoco-render', daemon=False)
        self.width, self.height, self.fps = width, height, fps
        self.api=api if api is not None else DataAPI()
        self.initial_model = model
        self.packages = dict(packages or {})
        self.events = SimpleQueue() if events else _DiscardEvents()
        self.initial_running = running
        self.continuous = continuous
        self.fallback = fallback
        self.ready = Event()
        self.failure = None
        self._request_lock = Lock()
        self.stop = Event()
        self.commands = Queue(maxsize=64)
        self.mailbox = LatestBatch()
        self.error = None
        self.gl_renderer = 'initializing'
        self.rendered = 0
        self.measurement = Measurement()

    def request(self, command, payload=None):
        if current_thread() is self:
            raise RuntimeError('Cannot make a blocking request from the runtime thread')
        future = Future()
        with self._request_lock:
            if self.stop.is_set() or not self.is_alive():
                raise RuntimeError(self.error or 'CameraLab is closed')
            self.commands.put_nowait(('_request', _Request(command, payload, future)))
        return future.result()

    def run(self):
        session = None
        request = None
        try:
            try:
                project = (SceneProject.open(self.initial_model,self.packages)
                           if self.initial_model else SceneProject.demo())
            except Exception as exc:
                if not self.fallback:raise
                project = SceneProject.demo()
                self.events.put({'ok':False,'message':f'{exc}\nShowing demo scene; choose Open to retry.'})
            session = RenderSession(self.width,self.height,project=project)
            self.gl_renderer = session.gl_renderer
            self.events.put({'scene':session.scene_description()})
            simulation=Simulation(session,running=(self.initial_model is None if self.initial_running is None else self.initial_running))
            self.events.put({'mode':simulation.mode.value})
            self.api.bind(session)
            self.ready.set()
            previous = perf_counter()
            while not self.stop.is_set():
                start = perf_counter()
                delta, previous = min(start-previous,.25),start
                manual_seconds=0.0
                while not self.stop.is_set():
                    try:
                        command,payload = self.commands.get_nowait()
                    except Empty:
                        break
                    request = None
                    result = None
                    if command == '_request':
                        request = payload
                        command, payload = request.command, request.payload
                    if self.stop.is_set():break
                    try:
                        if command == 'running':
                            simulation.transition('run' if payload else 'pause')
                            self.events.put({'mode':simulation.mode.value})
                            delta = 0
                        elif command == 'frustums':
                            session.show_frustums=payload
                        elif command in ('reset','step','edit','save_initial'):
                            if command=='step':
                                self.api.bind(session)
                                self.api.drain_controls(session)
                            before=float(session.data.time)
                            count=payload if command=='step' and payload is not None else 1
                            if command=='step' and (isinstance(count,bool) or not isinstance(count,Integral) or count<1):
                                raise ValueError('steps must be a positive integer')
                            for _ in range(count):
                                if self.stop.is_set():raise RuntimeError('CameraLab closed during step')
                                simulation.transition(command)
                            if command=='step':manual_seconds+=float(session.data.time)-before
                            self.events.put({'mode':simulation.mode.value})
                            self.events.put({'scene':session.scene_description()})
                            delta = 0
                        elif command == 'orbit':
                            session.orbit(*payload)
                        elif command == 'zoom':
                            session.zoom(payload)
                        elif command in ('edit_body','edit_camera','aim_camera','move_camera','configure_camera'):
                            if command == 'edit_body':
                                simulation.transition('pause')
                                self.events.put({'mode':simulation.mode.value})
                                delta = 0
                            try:
                                if command == 'configure_camera':
                                    fields=dict(payload)
                                    name=fields.pop('name')
                                    row=next((c for c in session.scene_description()['cameras'] if c['name']==name),None)
                                    if row is None:raise ValueError(f'Unknown camera: {name}')
                                    edits_pose=bool({'parent','position','rotation'} & fields.keys())
                                    parent=fields.pop('parent',row['parent'])
                                    position=fields.pop('position',row['position'])
                                    rotation=fields.pop('rotation',row['quaternion'])
                                    config=replace(row['config'],**fields)
                                    if edits_pose:
                                        session.edit_camera(name,parent,position,rotation,config)
                                    else:
                                        session.configure_camera(name,config)
                                elif command == 'aim_camera':
                                    bid=next((bid for bid,n in session._body_names.items() if n==payload['target']),None)
                                    if bid is None:raise ValueError(f"Unknown target: {payload['target']}")
                                    session.aim_camera(payload['name'],session.data.xpos[bid].copy())
                                else:
                                    getattr(session,command)(**payload)
                                if command=='edit_body':
                                    simulation.target_time=float(session.data.time)
                                    simulation.mode=Mode.EDIT
                                self.events.put({'mode':simulation.mode.value})
                                self.events.put({'scene':session.scene_description(),'edit_message':f'Applied; simulation {simulation.mode.value}.'})
                            except Exception as exc:
                                self.events.put({'scene':session.scene_description(),'edit_message':str(exc)})
                                if request is not None:raise
                        elif command == 'save_project':
                            simulation.transition('pause')
                            delta=0
                            self.events.put({'mode':simulation.mode.value})
                            try:
                                save_project(session,payload['path'],payload['layout'])
                                message='Saved project '+payload['path']
                            except Exception as exc:
                                message='Save failed: '+str(exc)
                                if request is not None:raise
                            self.events.put({'storage_message':message})
                            previous=perf_counter()
                        elif command in ('model','open_project'):
                            simulation.transition('pause')
                            self.events.put({'mode':simulation.mode.value})
                            delta = 0
                            replacement = None
                            try:
                                if command=='open_project':
                                    candidate,layout=load_project(payload['path'],revision=session.model_revision+1)
                                elif payload['append']:
                                    candidate = project.append(payload['path'],payload['position'],
                                                               payload['quaternion'],payload['packages'],parent=payload.get('parent','world'))
                                else:
                                    candidate = SceneProject.open(payload['path'],payload['packages'],
                                                                  revision=session.model_revision+1)
                                replacement = RenderSession(self.width,self.height,project=candidate)
                                if command=='model' and payload['append']:
                                    session.copy_edits_to(replacement)
                                    replacement.model_revision=session.model_revision+1
                                if command=='model':replacement.show_frustums=session.show_frustums
                                preview = replacement.capture()
                            except Exception as exc:
                                if replacement is not None:
                                    replacement.close()
                                self.events.put({'ok':False,'message':str(exc)})
                                if request is not None:raise
                            else:
                                old = session
                                session,project = replacement,candidate
                                old.close()
                                simulation=Simulation(session)
                                self.events.put({'mode':simulation.mode.value})
                                self.mailbox.put(preview)
                                message = f"Loaded {payload['path']} · model revision {candidate.revision}"
                                if candidate.warnings:
                                    message += '\n'+'\n'.join(candidate.warnings)
                                self.events.put({'ok':True,'message':message})
                                self.events.put({'scene':session.scene_description()})
                                if command=='open_project':self.events.put({'layout':layout,'frustums':session.show_frustums})
                            previous = perf_counter()
                        elif command == 'state':
                            from .sim.replay import state
                            result=state(session)
                        elif command == 'seek':
                            from .sim.replay import seek
                            seek(session,**payload)
                            simulation.transition('pause')
                            delta=0
                            self.events.put({'mode':simulation.mode.value})
                        elif command == 'capture':
                            self.api.bind(session)
                            self.api.drain_controls(session)
                            result = session.capture_sample()
                            self.api.publish(*result)
                        elif command == 'describe':
                            result = session.scene_description()
                        elif command == 'mode':
                            result = simulation.mode.value
                        else:
                            raise ValueError(f'Unknown command: {command}')
                        if request is not None:self.api.bind(session)
                    except Exception as exc:
                        if request is None:raise
                        request.future.set_exception(exc)
                    else:
                        if request is not None:
                            request.future.set_result(result)
                if self.stop.is_set():break
                self.api.bind(session)
                self.api.drain_controls(session)
                before_step = session.data.time
                simulation.advance(delta)
                if self.continuous or simulation.mode==Mode.RUNNING:
                    batch,truth = session.capture_sample()
                    self.api.publish(batch,truth)
                    self.measurement.add(batch.render_ms,session.data.time-before_step+manual_seconds)
                    self.mailbox.put(batch)
                    self.rendered += 1
                self.stop.wait(max(0,1/self.fps-(perf_counter()-start)))
        except Exception as exc:
            self.failure = exc
            self.error = f'{type(exc).__name__}: {exc}'
        finally:
            with self._request_lock:
                self.stop.set()
                if request is not None and not request.future.done():
                    request.future.set_exception(RuntimeError(self.error or 'CameraLab is closed'))
                while True:
                    try:
                        command, payload = self.commands.get_nowait()
                    except Empty:
                        break
                    if command == '_request':
                        payload.future.set_exception(RuntimeError(self.error or 'CameraLab is closed'))
            self.ready.set()
            self.api.close()
            if session is not None:
                session.close()
