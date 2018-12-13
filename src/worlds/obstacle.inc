# NOT RESOLVED, DON'T USE THIS
define obs_pose actuator
(
	# sctuator properties
	type "linear" # or rotational
	axis [1 1 0]
	
	min_position 	
	max_position 
	max_speed 
)

define obstacle position
(
	# actual size
	size [1.25 1 1]
	# centre of rotation offset
	origin [0.125 0 0 0]
	circle
	(
		
	)
	
	#positional things
	drive "diff"
	
	#obs_pose()
)

