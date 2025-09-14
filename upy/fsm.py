#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-19
# modified: 2021-07-09 (converted to micropython, made generic)
#
# IllegalStateError at bottom

from logger import Level

class FiniteStateMachine:
    '''
    Generic finite state machine (FSM) implementation.

    This class manages transitions between states defined by a provided state class
    and enforces allowed transitions as specified by a transition map. It supports
    registering callbacks that are called upon entering specific states.

    Notes:

    This is an abstract class, meant to be subclassed. Attempting to instantiate it
    directly will raise an exception.

    Callbacks registered for the initial state will *not* be invoked during
    construction. To ensure callbacks run for the initial state, you must
    explicitly trigger the transition to that state *after* registering callbacks,
    typically by calling `go_to(initial_state)` before performing further transitions.

    Example:

        fsm = FiniteStateMachine(logger, "task", State, initial_state=State.INITIAL, transition_map=transitions)
        fsm.on(State.INITIAL, callback_function)
        fsm.go_to(State.INITIAL)  # This will invoke registered callbacks

    Args:
        logger: Logger instance to use for logging.
        task_name (str): Identifier name for the FSM instance.
        state_class: Class representing the states; must contain state instances.
        initial_state: Initial state instance to start the FSM in.
        transition_map (dict, optional): Mapping of states to sets of allowed next states.
        level (optional): Logging level (not used internally here but reserved for future use).
    '''
    def __init__(self, logger, task_name, state_class, initial_state, transition_map=None, level=Level.INFO):
        '''
        Initialize the finite state machine.

        Args:
            logger:                           Logger object for logging debug and error messages.
            task_name (str):                  Name of the FSM task/instance.
            state_class:                      Class containing the state instances.
            initial_state:                    State instance representing the FSM start state.
            transition_map (dict, optional):  Dict mapping states to allowed next states.
            level (optional):                 Logging level (currently unused).
        '''
        if type(self) is FiniteStateMachine:
            raise TypeError("FiniteStateMachine is abstract and cannot be instantiated directly")
        self._log = logger
        self._task_name = task_name
        self._states = state_class
        self._callbacks = {}
        self._state = initial_state
        self._transitions = transition_map or {}
        self._log.debug("fsm initialised.")

    def on(self, state, callback):
        '''
        Register a callback function to be executed upon entering a specific state.

        Multiple callbacks can be registered per state and will be called
        in the order of registration.

        Args:
            state: The state instance for which to register the callback.
            callback: Callable with no arguments to be executed on entering `state`.
        '''
        if state not in self._callbacks:
            self._callbacks[state] = []
        self._callbacks[state].append(callback)

    def transition(self, next_state):
        '''
        Transition to the specified state.

        Args:
            next_state: The target state instance to transition into.
        '''
        self.__transition(next_state)

    def __transition(self, next_state):
        '''
        Attempt to transition to the specified next state.

        Checks if the transition is allowed per the transition map and raises
        IllegalStateError on invalid transitions. Executes registered callbacks
        for the new state after a successful transition.

        Args:
            next_state: The state instance to transition to.
        Raises:
            IllegalStateError: If the transition is not allowed.
        '''
        current = self._state
        self._log.debug("transition in {} from {} to {}.".format(self._task_name, current.code, next_state.code))
        valid_targets = self._transitions.get(current, set())
        if next_state not in valid_targets:
            raise IllegalStateError("invalid transition in {} from {} to {} (allowed: {}).".format(
                    self._task_name, current.code, next_state.code, ", ".join(s.code for s in valid_targets)))
        self._state = next_state
        # execute any callbacks for the new state
        for callback in self._callbacks.get(next_state, []):
            try:
                callback()
            except Exception as e:
                self._log.error("callback for {} failed: {}".format(next_state.code, e))

    @property
    def state(self):
        '''
        Returns:
            The current state instance of the FSM.
        '''
        return self._state

class IllegalStateError(RuntimeError):
    '''
    Exception raised when an invalid state transition is attempted.
    '''
    def __init__(self, message):
        super().__init__(message)

#EOF
