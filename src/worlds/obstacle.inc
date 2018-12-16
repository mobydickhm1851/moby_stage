define obstacle position
(
	# actual size
	size [1.25 1 1]
	# centre of rotation offset
	origin [0.125 0 0 0]
 	# octagon obstacle	


 block
      (
            points 6
            point[5] [1 0]
            point[4] [3 0]
            point[3] [4 1.732]
            point[2] [3 3.464]
            point[1] [1 3.464]
            point[0] [0 1.732]
            z [0 1]
      )
	
	#positional things
	drive "diff"
	localization "odom"
	ranger_return 1.0
        velocity_bounds [-5 5 -1 1 -1 1 -90 90 ]
	
	update_interval 100
)

