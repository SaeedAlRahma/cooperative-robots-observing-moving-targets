# Holonomic Point Targets
define holtarget position
(
	# actual size [x y z] in meters
	size [1 1 1]
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
	holtarget_eyes()
)

define holtarget_eyes blobfinder
(
	colors_count 1
	colors ["blue"]

	image [300 300]
	range 30.00
	fov 360
)

