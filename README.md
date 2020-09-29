# robil_lihi
This package is an addition to Robil2 packages.
Go to src->Simulation ans move the package "src" to robil_lihi
Move packages Framework and 3party to robil_lihi

In order to run it you should download gym:
```
sudo pip install gym
cd ~
git clone https://github.com/erlerobot/gym-gazebo
cd gym-gazebo
sudo pip install -e .
```
In addition, you need to install pytorch (for PPO) and TensorFlow2 (for DQN)

  
Run simulation:
```
roslaunch robil_lihi robil_lihi.launch
```
![Bobcat](https://github.com/Lihikal/robil_lihi_GIBUI/blob/master/Pictures/bobcat_and_pile.png)

**Run Q-learning training** 
```
rosrun robil_lihi train_Q_learning.py
```
**Run DQN training** 
```
rosrun robil_lihi train_DQN.py
```
observation space and action space are discrete. 
The actions are engine commands: 1. Bobcat velocity (X axis direction)
                                 2. Arm lifting velocity.
                                 3. Bucket upward rotation velocity.
                                 4. Arm lifting and bobcat velocity combined.
                                 
**Run PPO training**  

To get started, I recommend cloning [Chapter 19](https://github.com/PacktPublishing/Deep-Reinforcement-Learning-Hands-On-Second-Edition/tree/master/Chapter19)  of the code repository from [Deep Reinforcement Learning Hands-On Second Edition](https://www.zbukarf1.ga/book.php?id=xKdhDwAAQBAJ,) book.
observation space and action space are continuous. 
```                             
rosrun robil_lihi train_ppo.py -n ppo -e MovingBobcat-v0 --cuda
```  
If you dont have a GPU you should omit " --cuda"

In orfer to check you results:
```                             
python /home/FULL_ADDRESS/play_best.py -e MovingBobcat-v0 -m /home/robil/catkin_ws/saves/DAT_BEST_FILE.dat
``` 
where FULL_ADDRESS and DAT_BEST_FILE should be replaced.
