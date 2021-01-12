# Roman's Pybullet Gym Environments

This is where I keep my custom OpenAI Gym environments. To use them, enter the following commands from the terminal:

```
cd <path/to/where/you/want/to/store/these/files>
git clone https://github.com/roman-aguilera/romans_pybullet_gym_envs.git
cd romans_pybullet_gym_envs
pip install -e .
```
##URDF Files
* URDF Files can be found here (make sure to add them to your pybullet datapath):
https://github.com/roman-aguilera/romans_urdf_files/tree/master/octopus_files/python_scripts_edit_urdf

## Environments
The exact environments that I created myself are:
* The 8-link octopus environment:
https://github.com/roman-aguilera/romans_pybullet_envs/blob/master/octopus_env.py
* The 8-link octopus environment, with joint locking (work in rogress):
https://github.com/roman-aguilera/romans_pybullet_envs/blob/master/octopus_lockedjoints_env.py

## Requirements
PyKDL (used for locked env). Docs and references included here:

https://anaconda.org/conda-forge/python-orocos-kdl

https://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/

http://docs.ros.org/diamondback/api/kdl/html/python/index.html

http://wiki.ros.org/python_orocos_kdl

http://docs.ros.org/indigo/api/orocos_kdl/html/classKDL_1_1Rotation.html

https://github.com/orocos/orocos_kinematics_dynamics

ArgParse

https://docs.python.org/3.3/library/argparse.html

## Links for cleaning up later:
adding a gitignore file for swp files

* https://github.com/github/gitignore/blob/master/Global/Vim.gitignore

* https://github.com/github/gitignore/blob/master/Global/Vim.gitignore



## Credits:
This repository is a modified version of pybullet_envs: https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_envs 
The Pybullet Gymperium code was a helpful reference when designing these environments: https://github.com/benelot/pybullet-gym

