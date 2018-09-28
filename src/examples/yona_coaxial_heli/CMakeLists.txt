
px4_add_module(
	MODULE examples__yona_coaxial_heli
	MAIN yona_coaxial_heli
	STACK_MAIN 2000
	SRCS
		yona_coaxial_heli.cpp
	DEPENDS
	)
