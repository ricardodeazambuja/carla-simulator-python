from time import sleep
from subprocess import Popen, PIPE, TimeoutExpired
from threading import Thread
import psutil
import carla
"""Manage carla server container and external processes

It's up to the external process to gracefully die before procs_timeout... or it will be killed.
When external_processes=[] the server will be killed and relaunched and the external processes
will have to detect that, but my experience tell me the API will crash if you just keep creating 
new carla.Clients inside the same script.
"""

class AutomateCarla:
    def __init__(self, carla_timeout=10, procs_timeout=10, external_processes=[]):
        command = "docker run --rm --name carla-container --hostname=carla-container --user carla -p 2000-2002:2000-2002 --gpus 0 ricardodeazambuja/carlasim:0.9.13_headless /bin/bash -c"
        command = command.split(" ")
        self.command = command + ['sudo avahi-daemon -D && unset SDL_VIDEODRIVER && ./CarlaUE4.sh -vulkan -RenderOffscreen -nosound']
        self.kill_command = "docker kill carla-container".split(" ")

        self.external_processes = [e.split(" ") for e in external_processes]
        self.carla_timeout = carla_timeout # how long to wait to consider carla server frozen
        self.procs_timeout = procs_timeout # amount of time we give to the other processes to gracefully shutdown
        self.is_server_up = False
        self.shutdown = False

    def startexprocs(self):
        self.procs = []
        for s in self.external_processes:
            p = Popen(s, stdout=PIPE, stderr=PIPE)
            self.procs.append(p)
        self.procs

    def killprocs(self, procs, timeout=5):
        for p in procs:
            if p.poll() is None:
                try:
                    outs, errs = p.communicate(timeout)
                except TimeoutExpired:
                    p.kill()
                    if p.poll() != None:
                        self.kill(p.pid)
                    outs, errs = p.communicate()
                print(p.args)
                print(outs.decode())
                print(errs.decode())

    # from https://stackoverflow.com/a/25134985/7658422
    @staticmethod
    def kill(proc_pid):
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()


    @staticmethod
    def connect2carla(carla_timeout, maxtrials=2):
        trials = 0
        while trials<maxtrials:
            try:
                client = carla.Client('carla-container.local', 2000)
                client.set_timeout(carla_timeout)
                world = client.get_world()
                return client, world
            except RuntimeError as err:
                trials += 1
        raise RuntimeError("Can't connect...")


    def run(self):
        try:
            print(f"Launching a new carla container")
            carla_server = Popen(self.command, stdout=PIPE, stderr=PIPE)
            client, world = self.connect2carla(self.carla_timeout)
            self.is_server_up = True
            self.startexprocs()
            while carla_server.poll() is None and not self.shutdown:
                try:
                    # Carla server may "freeze" or slow down too much making the API timeout while the poll will still return None.
                    # Therefore world.wait_for_tick() will help raising the RuntimeError
                    world = client.get_world()
                    world.wait_for_tick()
                    sleep(0.5)
                except RuntimeError as err:
                    # when the simulator freezes, 
                    # it will raise this exception after timeout period
                    self.is_server_up = False
                    print(err)
                    try:
                        print(f"Kill current processes")
                        self.kill(carla_server.pid)
                        self.killprocs(self.procs, self.procs_timeout)
                    finally:
                        print(f"Kill current docker container")
                        # A docker container will not die easily... so we need to use docker kill
                        p2bkilled = Popen(self.kill_command, stdout=PIPE, stderr=PIPE)
                        out, err = p2bkilled.communicate()
                        print(out.decode(), err.decode())
                    
                    print(f"Launching a new carla container")
                    carla_server = Popen(self.command, stdout=PIPE, stderr=PIPE)
                    client, world = self.connect2carla(self.carla_timeout)
                    self.is_server_up = True
                    self.startexprocs()

            self.is_server_up = False
            print("Setting tasers to kill...")
            print(f"Kill all processes still alive")
            self.kill(carla_server.pid)
            self.killprocs(self.procs, self.procs_timeout)
        finally:
            print(f"Kill docker container")
            # A docker container will not die easily... so we need to use docker kill
            p2bkilled = Popen(self.kill_command, stdout=PIPE, stderr=PIPE)
            out, err = p2bkilled.communicate()
            print(out.decode(), err.decode())


if __name__ == "__main__":
    import os
    # path = os.path.dirname(os.path.realpath(__file__))
    # a = AutomateCarla(procs_timeout=20, external_processes=['python3 ' + path+'/getElevationSemantics.py'])

    # With an empty list for the external processes it will just relaunch carla server container
    a = AutomateCarla(procs_timeout=10, external_processes=[])

    ta = Thread(target=a.run)
    ta.start()
    try:
        i = 0
        while ta.is_alive():
            print(f"[{i:04d}] Is carla server up? {a.is_server_up}")
            i += 1
            sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        a.shutdown = True
        ta.join()
