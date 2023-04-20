# carla-simulator-python
My modifications to the examples from CARLA [**0.9.13**](https://carla.org/2021/11/16/release-0.9.13/) PythonAPI.



## Launch the simulator:
You can change ```--gpus 0``` to ```--gpus all``` or ```---gpus <some_number>``` if you have more than one GPU.
```
docker run --rm -it --name carla-container -u carla -p 2000-2002:2000-2002 --gpus 0 ricardodeazambuja/carlasim:0.9.13_headless ./launch_headless.sh
```
Press ENTER to kill the simulator (it takes some seconds until it dies...).


## Using the [PythonAPI](https://carla.readthedocs.io/en/0.9.13/python_api/)
```
docker cp carla-container:/home/carla/PythonAPI .
```

## Create a new conda environment (Python 3.7):
```
conda create --name carla-simulator python=3.7
```

```
conda activate carla-simulator
```

```
pip install --upgrade pip
```

```
cd PythonAPI/examples/
```

```
pip install -r requirements.txt 
```

### Install the carla python package (so we don't need to add the egg to the sys.path)
```
cd ../carla/dist/  #it's inside PythonAPI/carla/dist
```

```
pip install carla-0.9.13-cp37-cp37m-manylinux_2_27_x86_64.whl
```
*If pip complains and you are sure the Python version is correct, one possible cause is that you forgot to upgrade pip*

### The simulation is empty, so let's put something there!

```
python generate_traffic.py 
```
It should print ```spawned 30 vehicles and 10 walkers, press Ctrl+C to exit.```

### It's headless, so we need a way to see what's happening there
Open a new terminal (*don't forget to run ```conda activate carla-simulator```*) the and type:

```
python synchronous_mode.py
```
It should open a pygame display with a random vehicle moving through the town (with the focus on the pygame display, press ESC to exit).
