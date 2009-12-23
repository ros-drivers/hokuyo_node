alltwice:
	# Hack because dynamic_reconfigure currently has some cmake
	# problems.
	make all

include $(shell rospack find mk)/cmake.mk
