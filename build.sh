#!/bin/bash

component=$1

source toollib.bash

if [ -z "$component" ]; then
	echo 'usage: '$0' <component>'
	exit 1
fi

if [ "$component" = "all" ]; then

	# Add other components as needed.
	build_component kickerbot

else
	# Breaks if it's not a CMake thing.
	if [ -d "$component" ]; then
		build_component $component
	fi
fi
