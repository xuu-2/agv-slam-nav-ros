gnome-terminal --tab -- bash -c "source ~/.bashrc;
			   cd ~/rslidar_sdk/;
			   source ~/rslidar_sdk/devel/setup.bash;
			   roslaunch rslidar_sdk start.launch;
			   exec bash" #roscore" #新建终端启动节点
			   

gnome-terminal --tab -- bash -c "sleep 5;
                  source ~/.bashrc;
		   cd ~/hqcartest/;
		   source ~/hqcartest/devel/setup.bash;
		   roslaunch car_top cartographer_movebase.launch;
		   exec bash" #roscore" #新建终端启动节点	



gnome-terminal --tab -- bash -c "sleep 5;
                  source ~/.bashrc;
		   cd ~/cartographer/;
		   source ~/cartographer/devel_isolated/setup.bash;
		   roslaunch cartographer_ros demo_backpack_3d_localization.launch;
		   exec bash" #roscore" #新建终端启动节点


gnome-terminal --tab -- bash -c "sleep 5;
                  source ~/.bashrc;
		   cd ~/hqcartest/;
		   source ~/hqcartest/devel/setup.bash;
		   roslaunch nav move_base.launch;
		   exec bash" #roscore" #新建终端启动节点

                           

