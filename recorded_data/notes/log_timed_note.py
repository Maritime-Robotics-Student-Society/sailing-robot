#!/usr/bin/env python
"""Record a new timed note to associate with logs from the robot.

Symlink this script to a location on $PATH, e.g.:

    ln -s "$PWD/log_timed_note.py" ~/.local/bin/sailnote

Then run it when something weird happens that we want to be able to investigate
later. It will create files in this directory, which can be picked up by the
indexing tool later. These will be small, and may be committed to git.
"""
from __future__ import print_function
from datetime import datetime
import json
import os.path
import sys

if sys.version_info[0] < 3:
    input = raw_input

# Not using .isoformat() because we don't need milliseconds
now = datetime.utcnow().strftime("%Y-%m-%dT%H.%M.%SZ")
print("Logging note at", now)

this_dir = os.path.dirname(os.path.realpath(__file__))
filename = os.path.join(this_dir, now + '.json')
#print("Saving to:", filename)

msg = input('message: ')

# JSON may seem pointless for this, but if we think of extra information to add,
# we have space to put it.
with open(filename, 'w') as f:
    json.dump({'message': msg}, f, indent=2)

print('Saved! Make a git commit when you have a chance.')
