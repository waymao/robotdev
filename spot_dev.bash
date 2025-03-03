#!/bin/bash
tmux new-session  \; \
	send-keys 'sourcespot ' $1 Enter y Enter  \; \
	split-window -v \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	split-window -h \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	split-window -h \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	split-window -t %0 -h \; \
	send-keys 'sourcespot ' $1 Enter y Enter \; \
	resize-pane -t %1 -L 26 \; \
	resize-pane -t %2 -L 12 \; \
	wait-for -L lock1\; \
		send-keys -t %0 'sleep 3' Enter 'tmux wait-for -U lock1' Enter C-m \; \
		wait-for -L lock1 \; \
	send-keys -t %0 'launchspot' Enter 'launchspot' Enter \; \
	send-keys -t %4 'launchserver' Enter \; \
	wait-for -L lock2\; \
		send-keys -t %1 'sleep 8' Enter 'tmux wait-for -U lock2' Enter C-m \; \
		wait-for -L lock2 \; \
	send-keys -t %1 'allcams' Enter \; \
	send-keys -t %2 'alldepth' Enter \; \
	send-keys -t %3 'allext' Enter \; \
	set -g mouse on

