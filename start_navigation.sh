gnome-terminal --tab -- bash -c "sleep 5;
                  source ~/.bashrc;
		   cd ~/hqcartest/;
		   source ~/hqcartest/devel/setup.bash;
		   roslaunch car_top car_top.launch;
		   exec bash" #roscore" #新建终端启动节点	



gnome-terminal --tab -- bash -c "source ~/.bashrc;
			   cd ~/rslidar_sdk/;
			   source ~/rslidar_sdk/devel/setup.bash;
			   roslaunch rslidar_sdk start.launch;
			   exec bash" #roscore" #新建终端启动节点
			   


#gnome-terminal --tab -- bash -c "sleep 5;
                  #source ~/.bashrc;
		   #cd ~/hqcartest/;
		   #source ~/hqcartest/devel/setup.bash;
		   #roslaunch pointcloud_to_laserscan point_to_scan.launch;
		   #exec bash" #roscore" #新建终端启动节点	
		   

gnome-terminal --tab -- bash -c "sleep 5;
                  source ~/.bashrc;
		   cd ~/hqcartest/;
		   source ~/hqcartest/devel/setup.bash;
		   roslaunch wit_ros_imu display_and_imu.launch;
		   exec bash" #roscore" #新建终端启动节点	



gnome-terminal --tab -- bash -c "sleep 5;
                  source ~/.bashrc;
		   cd ~/cartographer/;
		   source ~/cartographer/devel_isolated/setup.bash;
		   roslaunch cartographer_ros backpack_3d.launch;
		   exec bash" #roscore" #新建终端启动节点



#gnome-terminal --tab -- bash -c "sleep 5;
			   #source ~/.bashrc;
			  #cd ~/hqcartest/;
			   #source ~/hqcartest/devel/setup.bash;
			   #sudo chmod 777 /dev/ttyUSB0;
			   #123;
			   #roslaunch nav car_run.launch;
                          #exec bash" #新建终端启动节点
                           

