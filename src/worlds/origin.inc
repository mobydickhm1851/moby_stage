define origin position
(
	# actual size
	size [0.001 0.001 1]
	# centre of rotation offset
	origin [0 0 0 0]
 	# octagon obstacle	


 block
      (
            points 4
            point[3] [0 0]
            point[2] [0.1 0]
            point[1] [0.1 0.1]
            point[0] [0 0.1]
            z [0 1]
      )
	
	#positional things
	localization "odom"
	ranger_return 1.0
	
	update_interval 100
)

