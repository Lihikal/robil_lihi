# robil_lihi
This package is an addition to robil packages.

In order to run it you should download gym:

```
sudo pip install gym
cd ~
git clone https://github.com/erlerobot/gym-gazebo
cd gym-gazebo
sudo pip install -e .
```
Run simulation:
```
roslaunch robil_lihi bobby.launch
```

A menu will pop up. To start the training you should click on "Find Path".
The Q table will be saved in folder "training_xxx" when the episod is done.
  
