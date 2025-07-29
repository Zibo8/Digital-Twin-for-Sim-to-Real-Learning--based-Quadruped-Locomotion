# Digital-Twin-for-Sim-to-Real-Learning--based-Quadruped-Locomotion


## virtual env
recommand miniforge

```
cd isaacsim
```
conda-env:sim
```
conda env create -f environment.yml
```

conda-env:env-isaaclab
```
cd isaaclab
```

```
conda env create -f environment.yml
```

## bashrc

```
cd ~
```
```
gedit .bashrc
```
add these commands to bashrc file

```
conda activate sim
```
```
export PYTHONPATH=~/unitree_sdk2_python:$PYTHONPATH
```
```
export PYTHONPATH=~/in_ubuntu:$PYTHONPATH 
```
source

```
source .bashrc
```
## run_win
```
cd in_windows
```
```
python vel_and_command_average.py
```
## run_ubuntu_real
```
cd in_ubuntu
```
```
python real_go2/twin_go2.py
```
```
python real_go2/real_go2.py
```
## run_ubuntu_sim
```
python sim_go2/sim_go2.py
```