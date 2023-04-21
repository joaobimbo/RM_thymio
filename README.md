### Git repo for running Thymio on CoppeliaSim

- Part of 2022/2023 Mobile Robots class at [fc.ul.pt](https://www.fc.ul.pt/)
- Example model to go from $a$ to $b$ can be retrieved from [here](https://cirrus.ciencias.ulisboa.pt/owncloud/s/HjrBD8S6Xb5eiBg)
- Instructions:
  1. Open a simulation instance and load Arena_Empty.ttt
  2. Run RL_test and either:
    - download existing model and run it with ``model = DQN.load("move_robot_c")`` or
    - learn a new policy with ``model.learn(total_timesteps=100000, log_interval=1)``
  
-- Joao Bimbo
