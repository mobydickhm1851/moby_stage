define solabot_sonar sensor
(
    # define the size of each transducer [xsize ysize zsize] in meters
    size [0.01 0.05 0.01 ] 
    # define the range bounds [min max]
    range [0.3 2.0]
    # define the angular field of view (by default in degrees)
    fov 10
    # define the color that ranges are drawn in the gui
    color_rgba [ 0 1 0 0.5] 
    # how many rays in a beam
    samples 180
)

define solabot_sonars ranger
( 
  # one line for each sonar [xpos ypos zpos heading]
  bigbobs_sonar( pose [ 0.75 0.1875 0 0])
  bigbobs_sonar( pose [ 0.75 -0.1875 0 0])
  bigbobs_sonar( pose [ 0.25 0.5 0 30])
  bigbobs_sonar( pose [ 0.25 -0.5 0 -30])
)

define solabot_eyes blobfinder
(
	colors_count 2
	colors ["orange" "DarkBlue"]
	image [160 120] # resolution
	range 5.00 		 # meters
	fov 60 # fov (in unit_angle (degrees by def))
	)

# using hokuyo UGR-04LX-UG01
define solabot_laser sensor
(
    # define the size of each transducer [xsize ysize zsize] in meters
    size [0.025 0.025 0.025]
    # define the range bounds [min max]
    range [0 0.25]
    # define the angular field of view (by default in degrees)
    fov 240
    # define the color that ranges are drawn in the gui
    color_rgba [ 1 0 0 0.5] 
    samples 667
)

define solabot_lasers ranger
( 
  # one line for each laser [xpos ypos zpos heading]
  solabot_laser( pose [ 0.625 0.125 -0.975 270 ])
)

define solabot position
(
	  # actual size
	  size [1.2 1.8 0.5]
	  # centre of rotation offset
	  origin [0.0 0 0 0]

	  # the chassis of solabot
	  block
	  (
		 points 4
		 point[3] [1 1]
		 point[2] [5 1]
		 point[1] [5 8]
		 point[0] [1 8]
		 z [1 2.5]
	  )

	  # the left-rear wheel of solabot
	  block
	  (
		 points 4
		 point[3] [0 0]
		 point[2] [1 0]
		 point[1] [1 2]
		 point[0] [0 2]
		 z [0.0 2]
	  )

	  # the right-rear wheel of solabot
	  block
	  (
		 points 4
		 point[3] [5 0]
		 point[2] [6 0]
		 point[1] [6 2]
		 point[0] [5 2]
		 z [0.0 2]
	  )

	  # the right-front wheel of solabot
	  block
	  (
		 points 4
		 point[3] [5 7]
		 point[2] [6 7]
		 point[1] [6 9]
		 point[0] [5 9]
		 z [0.0 2]
	  )

	  # the left-front wheel of solabot
	  block
	  (
		 points 4
		 point[3] [0 7]
		 point[2] [1 7]
		 point[1] [1 9]
		 point[0] [0 9]
		 z [0.0 2]
	  )

	  #positional things
	  drive "omni"
	  localization "odom"
	  ranger_return 1.0
     	  update_interval 100
          velocity_bounds [-1 1 -5 5 -1 1 -90 90 ]

	  #sensors attached to solabot
#	   solabot_sonars( ) 
#      solabot_eyes( )
#  	   solabot_lasers( )
)

