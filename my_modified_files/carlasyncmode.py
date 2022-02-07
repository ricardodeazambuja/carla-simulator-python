#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import queue

import carla


class CarlaSyncMode():
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, sensor_list, **kwargs):
        self.world = world
        self.sensors = sensor_list
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings() # save the current world settings
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)

        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        # assert all(x.frame == self.frame for x in data) # some sensors may take too long and return None
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings) # restore the previous world settings

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            try:
                data = sensor_queue.get(timeout=timeout)
            except queue.Empty:
                return None # if a sensor takes too long, it will return None
            if data.frame == self.frame:
                return data