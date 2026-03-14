
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-03-12
# modified: 2026-03-14

import traceback
from math import degrees, radians, sqrt, atan2
from math import pi as π
from colorama import init, Fore, Style
init()

from core.component import Component, MissingComponentError
from core.logger import Logger, Level
from core.event import Event
from behave.async_behaviour import AsyncBehaviour
from hardware.motor_controller import MotorController
from hardware.movement_controller import MovementController
from hardware.rotation_controller import RotationController

class Navigator(AsyncBehaviour):
    NAME = 'navigator'
    '''
    Navigates a route defined as a list of waypoints. Each leg is executed
    by delegating to MovementController.run_vector(), which handles accel,
    constant-speed movement, and decel with slew-limited stopping. Navigator
    is solely responsible for route management; MovementController owns the
    intent vector for the duration of each leg.
    '''
    def __init__(self, config=None, message_bus=None, message_factory=None, level=Level.INFO):
        self._log = Logger(Navigator.NAME, level)
        _component_registry = Component.get_registry()
        _motor_controller = _component_registry.get(MotorController.NAME)
        if _motor_controller is None:
            raise MissingComponentError('motor controller not available.')
        self._odometer = _motor_controller.odometer
        self._movement_controller = _component_registry.get(MovementController.NAME)
        if self._movement_controller is None:
            self._log.info('creating rotation controller…')
            _rotation_controller = None
            self._log.info('creating movement controller…')
            self._movement_controller = MovementController(config, _motor_controller, _rotation_controller, level=level)
        AsyncBehaviour.__init__(self, self._log, config, message_bus, message_factory, _motor_controller, level=level)
        self._roam           = None # the Roam behaviour
        self._scout          = None # the Scout behaviour
        self._route          = None
        self._route_index    = 0
        self._door_callback  = None
        self._route_complete_callback = None
        self._en_route_callback = None
        # configuration
        _cfg = config['kros'].get('behaviour').get('navigator')
        self._verbose        = _cfg.get('verbose', False)
        self._priority       = _cfg.get('default_priority', 0.4)
        self._default_speed  = _cfg.get('default_speed', 0.5)
        self._log.info('ready.')

    def _get_roam(self):
        '''
        Lazily fetch the Roam behaviour from the component registry.
        '''
        if self._roam is None:
            from behave.roam import Roam
            self._roam = Component.get_registry().get(Roam.NAME)
        return self._roam

    def _get_scout(self):
        '''
        Lazily fetch the Scout behaviour from the component registry.
        '''
        if self._scout is None:
            from behave.scout import Scout
            self._scout = Component.get_registry().get(Scout.NAME)
        return self._scout

    def set_door_callback(self, callback):
        '''
        Set a callback invoked whenever the robot clears a door waypoint.
        The callback receives the Waypoint as its argument.
        '''
        self._door_callback = callback

    def set_route_complete_callback(self, callback):
        '''
        Set a callback invoked when the robot completes the route.
        The callback receives the Waypoint as its argument.
        '''
        self._route_complete_callback = callback

    def set_en_route_callback(self, callback):
        '''
        Set a callback invoked when the robot starts or completes a route.
        The callback receives the boolean status as its argument.
        '''
        self._en_route_callback = callback

    def set_route(self, waypoints, is_first_route=False):
        '''
        Set the route to navigate. If is_first_route is True, sets the odometer
        pose from waypoint 0 and begins from waypoint 1. Otherwise compares
        current pose to waypoint 0 and inserts a pre-navigation leg if needed.
        '''
        if not waypoints:
            self._log.warning('set_route: empty waypoint list.')
            return
        self._log.info('set route to waypoints: {}'.format(waypoints))
        self._route       = waypoints
        self._route_index = 0
        _wp0              = self._route[0]
        if is_first_route:
            _heading_deg = _wp0.heading if _wp0.heading is not None else 0.0
            if _wp0.heading is None:
                self._log.warning('no heading set for waypoint 0: defaulting to 0.0° (north).')
            self._odometer.set_pose(_wp0.position.x, _wp0.position.y,
                    radians(_heading_deg), use_radians=True)
            self._log.info('odometer pose set to waypoint 0: {}'.format(_wp0))
            self._route_index = 1
        else:
            _x, _y, _theta = self._odometer.pose
            if _wp0.reached(_x, _y):
                self._route_index = 1
                self._log.info('already at waypoint 0, starting from waypoint 1.')
            else:
                self._log.info('not at waypoint 0, will navigate there first.')
        # suppress Roam and Scout
        _roam = self._get_roam()
        if _roam is not None:
            _roam.suppress()
            self._log.info('roam suppressed for navigation.')
        else:
            self._log.warning('roam not available to suppress.')
        _scout = self._get_scout()
        if _scout is not None:
            _scout.suppress()
            self._log.info('scout suppressed for navigation.')
        else:
            self._log.warning('scout not available to suppress.')
        if self._en_route_callback is not None:
            self._en_route_callback(True)
        self._log.info('route set: {} waypoints, starting from index {}.'.format(
                len(self._route), self._route_index))

    @property
    def is_ballistic(self):
        return False

    def callback(self):
        raise NotImplementedError('callback unsupported in Navigator.')

    def execute(self, message):
        raise NotImplementedError('execute unsupported in Navigator.')

    def start_loop_action(self):
        pass

    def stop_loop_action(self):
        pass

    async def _poll(self):
        '''
        The asynchronous poll, drives navigation by executing one waypoint leg
        at a time via MovementController.
        '''
        if self.suppressed or self.disabled:
            return (0.0, 0.0, 0.0)
        try:
            await self._navigate_to_waypoint()
        except Exception as e:
            self._log.error("{} thrown while polling: {}\n{}".format(type(e), e, traceback.format_exc()))
            self.disable()
        return (0.0, 0.0, 0.0)

    async def _navigate_to_waypoint(self):
        '''
        Computes the displacement from the current pose to the current waypoint
        and delegates movement to MovementController via run_vector(). Advances
        the route index when the waypoint is reached. Returns immediately if no
        route or route is complete.

        For multi-leg routes, omega is computed as the bearing change required
        to face the next waypoint at the end of the current leg, giving a smooth
        heading transition into the next leg. For the final leg omega is zero.
        '''
        if not self._route or self._route_index >= len(self._route):
            return
        _x, _y, _theta = self._odometer.pose
        _wp = self._route[self._route_index]
        if _wp.reached(_x, _y):
            self._log.info('waypoint {} reached: {}'.format(self._route_index, _wp.label))
            if _wp.kind == 'door' and self._door_callback is not None:
                _degrees = degrees(_theta)
                _rad_pi  = _theta / π
                self.console('CYAN pose at doorway {}:\nCYAN   x:YELLOW          {:d}mm\nCYAN   y:YELLOW          {:d}mm\nCYAN   theta:YELLOW      {:.3f}π ({:.1f}°)'.format(
                        _wp,
                        round(_x),
                        round(_y),
                        _rad_pi,
                        _degrees))
                self._door_callback(_wp)
            self._route_index += 1
            if self._route_index >= len(self._route):
                self._log.info('route complete.')
                if self._route_complete_callback is not None:
                    self._route_complete_callback(_wp)
                if self._en_route_callback is not None:
                    self._en_route_callback(False)
                self._route = None
                _roam = self._get_roam()
                if _roam is not None:
                    _roam.release()
                    self._log.info('roam released: route complete.')
                _scout = self._get_scout()
                if _scout is not None:
                    _scout.release()
                    self._log.info('scout released: route complete.')
                return
            _wp = self._route[self._route_index]
            self._log.info('advancing to waypoint {}: {}'.format(self._route_index, _wp.label))
        # compute displacement from current pose to waypoint
        _dx   = _wp.position.x - _x
        _dy   = _wp.position.y - _y
        _dist = sqrt(_dx * _dx + _dy * _dy)
        if _dist < 1e-6:
            return
        _vx        = _dx / _dist
        _vy        = _dy / _dist
        # omega: bearing change needed to face the next waypoint at end of this leg
        if self._route_index + 1 < len(self._route):
            _next_wp     = self._route[self._route_index + 1]
            _next_bear   = atan2(_next_wp.position.x - _wp.position.x,
                                 _next_wp.position.y - _wp.position.y)
            _this_bear   = atan2(_dx, _dy)
            _heading_err = _next_bear - _this_bear
            while _heading_err > π:
                _heading_err -= 2.0 * π
            while _heading_err < -π:
                _heading_err += 2.0 * π
            _omega = _heading_err / π
        else:
            _omega = 0.0
        self._log.info('navigating to waypoint {}: dist={:.1f}mm, vx={:.3f}, vy={:.3f}, omega={:.3f}'.format(
                self._route_index, _dist, _vx, _vy, _omega))
        await self._movement_controller.run_vector(_dist, (_vx, _vy, _omega))

    async def x_navigate_to_waypoint(self):
        '''
        Computes the displacement from the current pose to the current waypoint
        and delegates movement to MovementController via run_vector(). Advances
        the route index when the waypoint is reached. Returns immediately if no
        route or route is complete.
        '''
        if not self._route or self._route_index >= len(self._route):
            return
        _x, _y, _theta = self._odometer.pose
        _wp = self._route[self._route_index]
        if _wp.reached(_x, _y):
            self._log.info('waypoint {} reached: {}'.format(self._route_index, _wp.label))
            if _wp.kind == 'door' and self._door_callback is not None:
                _degrees = degrees(_theta)
                _rad_pi  = _theta / π
                self.console('CYAN pose at doorway {}:\nCYAN   x:YELLOW          {:d}mm\nCYAN   y:YELLOW          {:d}mm\nCYAN   theta:YELLOW      {:.3f}π ({:.1f}°)'.format(
                        _wp,
                        round(_x),
                        round(_y),
                        _rad_pi,
                        _degrees))
                self._door_callback(_wp)
            self._route_index += 1
            if self._route_index >= len(self._route):
                self._log.info('route complete.')
                if self._route_complete_callback is not None:
                    self._route_complete_callback(_wp)
                if self._en_route_callback is not None:
                    self._en_route_callback(False)
                self._route = None
                _roam = self._get_roam()
                if _roam is not None:
                    _roam.release()
                    self._log.info('roam released: route complete.')
                _scout = self._get_scout()
                if _scout is not None:
                    _scout.release()
                    self._log.info('scout released: route complete.')
                return
            _wp = self._route[self._route_index]
            self._log.info('advancing to waypoint {}: {}'.format(self._route_index, _wp.label))
        # compute displacement from current pose to waypoint
        _dx   = _wp.position.x - _x
        _dy   = _wp.position.y - _y
        _dist = sqrt(_dx * _dx + _dy * _dy)
        if _dist < 1e-6:
            return
        # normalise translational components
        _vx = _dx / _dist
        _vy = _dy / _dist
        # heading error as normalised omega
        _bearing     = atan2(_dx, _dy)  # bearing to waypoint in map frame (north=0)
        _heading_err = _bearing - _theta
        while _heading_err > π:
            _heading_err -= 2.0 * π
        while _heading_err < -π:
            _heading_err += 2.0 * π
        _omega = _heading_err / π
        self._log.info('navigating to waypoint {}: dist={:.1f}mm, vx={:.3f}, vy={:.3f}, omega={:.3f}'.format(
                self._route_index, _dist, _vx, _vy, _omega))
        await self._movement_controller.run_vector(_dist, (_vx, _vy, _omega))

    def enable(self):
        if not self.enabled:
            self._log.debug('enabling navigator…')
            super().enable()
            self._log.info('navigator enabled.')
        else:
            self._log.warning('already enabled.')

    def disable(self):
        if self.enabled:
            self._log.debug('disabling navigator…')
            super().disable()
            self._log.info('disabled.')
        else:
            self._log.warning('already disabled.')

#EOF
