# robil_lihi
This package is an addition to Robil packages.

In order to run it you should download gym:

```
sudo pip install gym
cd ~
git clone https://github.com/erlerobot/gym-gazebo
cd gym-gazebo
sudo pip install -e .
```
In addition, you need to install pytorch (for PPO) and TensorFlow2 (for DQN)

**Run Q-learning/DQN training**   
Run simulation:
```
roslaunch robil_lihi bobby.launch
```
A menu will pop up. To start the training you should click on "Find Path - DQN".
The Bobcat simulation start to train the robot using Deep Reinforcement Learning algorithm.
A single state is composed of 5 elements of the bucket: X position, Z position, robot x velocity, arm velocity and bucket velocity.

![Bobcat in simulation] (/Pictures/bobcat_and_pile.png)

We divided the screen into a 6x6 grid according to the bucket position. The bucket and arm velocity divided to 3 options and the
bobcat velocity to 4. 
The actions are engine commands: 1. Bobcat velocity (X axis direction)
                                 2. Arm lifting velocity.
                                 3. Bucket upward rotation velocity.
                                 4. Arm lifting and bobcat velocity combined.
                                 
**Run PPO training**                                 
To get started, I recommend cloning "Chapter 19"  of the code repository from the [Deep Reinforcement Learning Hands-On Second Edition!] (https://www.zbukarf1.ga/book.php?id=xKdhDwAAQBAJ,) book.
```                             
rosrun robil_lihi train_ppo.py -n ppo -e MovingBobcat-v0 --cuda
```  
If you dont have a GPU you should omit " --cuda"
