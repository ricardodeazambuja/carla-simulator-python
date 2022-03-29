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
    Context manager to synchronize output from different sensors. 
    When master=True (default), synchronous mode is enabled as long 
    as we are inside this context.

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, sensor_list, master=True, **kwargs):
        self.world = world
        self.sensors = sensor_list
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None
        self.master = master

    def __enter__(self):
        self._settings = self.world.get_settings() # save the current world settings
        if self.master:
            self.frame = self.world.apply_settings(carla.WorldSettings(
                no_rendering_mode=False,
                synchronous_mode=True,
                fixed_delta_seconds=self.delta_seconds))
        else:
            self.fixed_delta_seconds = self.world.get_settings().fixed_delta_seconds
            if not self.fixed_delta_seconds:
                raise ValueError("You need a master ticking the world!")

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick) # snapshot

        for sensor in self.sensors:
            make_queue(sensor.listen)

        return self

    def tick(self, timeout=None):
        if self.master:
            if timeout == None:
                raise ValueError("timeout must be specified for master=True")
            self.frame = self.world.tick()
        else:
            if timeout < self.fixed_delta_seconds:
                timeout = self.fixed_delta_seconds
            self.frame = self.world.wait_for_tick(seconds=timeout)

        data = [self._retrieve_data(i, q, timeout) for i,q in enumerate(self._queues)]
        # assert all(x.frame == self.frame for x in data) # some sensors may take too long and return None
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings) # restore the previous world settings

    def _retrieve_data(self, i, sensor_queue, timeout):
        trials = 0
        while trials<3:
            trials += 1
            try:
                data = sensor_queue.get(timeout=timeout)
            except queue.Empty:
                continue #return None # if a sensor takes too long, it will return None

            if self.master:
                if (self.frame-data.frame)<=1:
                    return data
            else:
                if (self.frame.frame-data.frame)<=1:
                    return data
        return None