"""Explicit UI state transitions; all operations run on the render owner thread."""
from enum import Enum


class Mode(str,Enum):
    EDIT='edit'
    PAUSED='paused'
    RUNNING='running'


class Simulation:
    def __init__(self,session,running=False):
        self.session=session
        self.mode=Mode.RUNNING if running else Mode.EDIT
        self.target_time=float(session.data.time)

    def transition(self,action,expected_run_id=None):
        if expected_run_id is not None and expected_run_id!=self.session.run_id:
            raise ValueError('Command is stale: run has changed')
        if action=='run':self.mode=Mode.RUNNING
        elif action=='pause':self.mode=Mode.PAUSED
        elif action=='step':
            self.session.single_step();self.mode=Mode.PAUSED
        elif action in ('reset','edit'):
            self.session.reset();self.mode=Mode.EDIT
        elif action=='save_initial':
            self.session.save_initial();self.mode=Mode.EDIT
        else:raise ValueError(f'Unknown simulation action: {action}')
        self.target_time=float(self.session.data.time)

    def advance(self,seconds):
        if self.mode==Mode.RUNNING:
            self.target_time+=seconds
            self.session.advance_to(self.target_time)
