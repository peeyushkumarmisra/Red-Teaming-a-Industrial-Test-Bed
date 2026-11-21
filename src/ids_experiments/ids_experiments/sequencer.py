from dataclasses import dataclass

import numpy as np

from ids_experiments.trajectory import quintic


@dataclass(frozen=True)
class Move:
    target: tuple
    duration: float
    label: str = ''


@dataclass(frozen=True)
class Dwell:
    duration: float
    label: str = ''


@dataclass(frozen=True)
class SetContext:
    context: int
    mass: float
    label: str = ''


@dataclass(frozen=True)
class Command:
    name: str
    label: str = ''
    args: tuple = ()


@dataclass
class ScanOutput:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    context: int
    mass: float
    commands: list
    step: int
    label: str
    finished: bool


class Sequencer:

    def __init__(self, start, program, passes=1):
        self._hold = np.asarray(start, dtype=float).copy()
        self._from = self._hold.copy()
        self._program = list(program)
        self._passes = passes
        self._completed = 0
        self._index = 0
        self._started_at = None
        self._context = 0
        self._mass = 0.0
        self._finished = not self._program

    @property
    def completed_passes(self):
        return self._completed

    @property
    def finished(self):
        return self._finished

    def scan(self, t):
        zero = np.zeros_like(self._hold)
        position, velocity, acceleration = self._hold.copy(), zero.copy(), zero.copy()
        commands = []
        label = ''
        index = self._index
        if not self._finished:
            step = self._program[self._index]
            label = step.label
            if self._started_at is None:
                self._started_at = t
                self._from = self._hold.copy()
                if isinstance(step, Command):
                    commands.append(step)
                elif isinstance(step, SetContext):
                    self._context = step.context
                    self._mass = step.mass if step.context else 0.0
            elapsed = t - self._started_at
            done = True
            if isinstance(step, Move):
                position, velocity, acceleration = quintic(self._from, step.target,
                                                           step.duration, elapsed)
                done = elapsed >= step.duration - 1e-9
                if done:
                    self._hold = np.asarray(step.target, dtype=float).copy()
            elif isinstance(step, Dwell):
                done = elapsed >= step.duration - 1e-9
            if done:
                self._advance()
        return ScanOutput(position, velocity, acceleration, self._context, self._mass, commands,
                          index, label, self._finished)

    def _advance(self):
        self._started_at = None
        self._index += 1
        if self._index >= len(self._program):
            self._index = 0
            self._completed += 1
            if self._passes and self._completed >= self._passes:
                self._finished = True


def transfer_program(source, destination, mass, travel_time, approach_time, dwell, place=None):
    program = [
        Move(tuple(source['hover']), travel_time, 'to source hover'),
        Move(tuple(source['grab']), approach_time, 'descend to grab'),
        Dwell(dwell, 'settle at grab'),
        SetContext(1, mass, 'payload on'),
        Command('attach', 'attach'),
        Move(tuple(source['hover']), approach_time, 'lift'),
        Move(tuple(destination['hover']), travel_time, 'transfer'),
        Move(tuple(destination['drop']), approach_time, 'lower'),
        SetContext(0, 0.0, 'payload off'),
        Command('detach', 'release'),
        Dwell(dwell, 'settle release'),
    ]
    if place is not None:
        program.append(Command('seat', 'seat part', tuple(place)))
    program.append(Move(tuple(destination['hover']), approach_time, 'retreat'))
    return program


def shuttle_program(a, b, mass, travel_time, approach_time, dwell, places=None):
    to_b, to_a = places if places is not None else (None, None)
    return (transfer_program(a, b, mass, travel_time, approach_time, dwell, to_b)
            + transfer_program(b, a, mass, travel_time, approach_time, dwell, to_a))