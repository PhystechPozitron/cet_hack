
RPS is relative positioning system for project "copter_show"

Nowadays it consists of 2 modules:

1) "rps_pose.py" is aimed to calculate IMU's position in 'rps_map', using video stream from the main camera. 
You can change the following parameters in file clever/launch/rps.launch:
	"cinfo_path" - path to .yaml file with information about the camera (camera matrix and distortion coefficients)
	"first_marker" - id of the first marker in board
	"markers_number" - number of markers in board
	"markers_dist" - distance between markers in board (m)
	"markers_side" - length of marker's side (m)

Notice that you can change rate of the main camera's stream in clever/launch/main_camera.launch, name of node - "cam_throttle" 

2) "rps_offboard.py" is aimed to afford services to control the vehicle (TODO)