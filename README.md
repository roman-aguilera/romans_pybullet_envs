# Roman's Pybullet Gym Environments

This is where I keep my custom OpenAI Gym environments. To use them, enter the following commands from the terminal:

```
cd <path/to/where/you/want/to/store/these/files>
git clone https://github.com/roman-aguilera/romans_pybullet_gym_envs.git
cd romans_pybullet_gym_envs
pip install -e .
```



The exact environments that I created myself are:
* The 8-link octopus environment:
https://github.com/roman-aguilera/romans_pybullet_envs/blob/master/octopus_env.py
* The 8-link octopus environment, with joint locking (work in rogress):
https://github.com/roman-aguilera/romans_pybullet_envs/blob/master/octopus_lockedjoints_env.py


## Links for cleaning up later:
adding a gitignore file

* https://github.com/github/gitignore/blob/master/Global/Vim.gitignore

* https://github.com/github/gitignore/blob/master/Global/Vim.gitignore



## Credits:
This repository is a modified version of pybullet_envs: https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_envs 
The Pybullet Gymperium code was a helpful reference when designing these environments: https://github.com/benelot/pybullet-gym

