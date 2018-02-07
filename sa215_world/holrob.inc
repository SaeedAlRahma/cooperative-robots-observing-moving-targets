# Holonomic Point Robot
define holrob position
(
	# actual size [x y z] in meters
	size [2 2 2]
	# center of rotation offset
	origin [0 0 0 0]
	
	# shape [x y] in meters fit inside size above
	block
	(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)

	# positonal things
	drive "omni"

	# sensors attached
	holrobs_ranger()
)

define holrobs_sonar sensor (
	# size [x y z] in meters
	size [0.1 0.1 0.1]
	# range bounds [min max] in meters
	range [0 30.0]
	# angular fiel of view in degrees
	fov 360
	# sensor color in GUI
	color_rgba [0 1 0 1]
)

define holrobs_ranger ranger (
	# position of each sonar [x y z heading] in meters and degrees
	holrobs_sonar (
		pose [0 0 0 0]
	)
)
