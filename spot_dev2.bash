#!/bin/bash
tmux new-session  \; \
	send-keys 'sourcespot ' $1 Enter y Enter  \; \
	split-window -v \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	split-window -h \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	split-window -h \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	split-window -t %5 -h \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	resize-pane -t %6 -L 26 \; \
	resize-pane -t %7 -L 12 \; \
    send-keys -t %5 'launchspot2' Enter 'launchspot2' Enter \; \
	send-keys -t %6 'allcams 2' Enter \; \
	send-keys -t %7 'alldepth 2' Enter \; \
	send-keys -t %8 'allext 2' Enter \; \
	send-keys -t %9 'rosrun spot_driver compute_spot_tf' Enter \; \
	set -g mouse

