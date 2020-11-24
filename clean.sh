#!/bin/bash

source toollib.bash

action=clean_component
if [ "$1" = "--purge" ]; then
	action=purge_component
	shift 1
fi

comp=$1

if [ -z "$comp" ] || [ "$comp" = "--all" ]; then

	# Add more components as necessary.
	$action kickerbot

else
	if [ -d "objdir.$comp" ]; then
		$action $comp
	else
		echo 'invalid component'
	fi
fi
