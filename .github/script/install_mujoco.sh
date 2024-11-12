#/usr/bin/bash
sudo apt-get -y install libglfw3-dev libglew-dev

cd $HOME
apt-get install wget
wget https://github.com/deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz
tar -xf mujoco-3.2.3-linux-x86_64.tar.gz
