# ttymidi
ttymidi supporting system exclusive MIDI messages
The original ttymidi application did not support sysex messages. This version does.
Based it on the work of johnty/ttymidi-icubex.c (see https://gist.github.com/johnty/de8b3d3041c7ee43accd)
to compile: gcc ttymidi.c -o ttymidi -lasound -pthread
Developed on Raspbian GNU/Linux 8 (jessie)
