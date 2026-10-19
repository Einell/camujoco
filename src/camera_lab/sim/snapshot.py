"""Immutable in-memory integration snapshots, scoped to one model and run."""
from dataclasses import dataclass
import numpy as np
import mujoco
from ..contracts import readonly

STATE = mujoco.mjtState.mjSTATE_INTEGRATION


@dataclass(frozen=True)
class Snapshot:
    run_id: str
    model_revision: int
    step_index: int
    state: np.ndarray


def capture(session):
    state=np.empty(mujoco.mj_stateSize(session.model,STATE))
    mujoco.mj_getState(session.model,session.data,state,STATE)
    return Snapshot(session.run_id,session.model_revision,session.step_index,readonly(state))


def restore(session,snapshot):
    if (snapshot.run_id!=session.run_id or snapshot.model_revision!=session.model_revision):
        raise ValueError('Snapshot is stale or belongs to a different session')
    size=mujoco.mj_stateSize(session.model,STATE)
    if snapshot.state.shape!=(size,) or not np.isfinite(snapshot.state).all():
        raise ValueError('Invalid snapshot state')
    mujoco.mj_setState(session.model,session.data,snapshot.state,STATE)
    mujoco.mj_forward(session.model,session.data)
    # Forward may update solver warm-start data; preserve the exact saved state.
    mujoco.mj_setState(session.model,session.data,snapshot.state,STATE)
