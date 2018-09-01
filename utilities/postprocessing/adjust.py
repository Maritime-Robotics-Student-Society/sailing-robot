"""Adjust the timestamps of recorded data files.

For when we've done a test with the clock on the raspi set incorrectly.
"""

from datetime import datetime, timedelta
from pathlib import Path
import re

# CORRECTION = timedelta(10, 60496)
CORRECTION = timedelta(seconds=(1535393272-1529482132)) # epoch from good - raspi
TIMESTAMP_RE = r'(\d{4})-(\d{2})-(\d{2})-?T?(\d{2})[-.](\d{2})[-.](\d{2})'

#for path in sorted(Path.cwd().glob('*_2018-06-20-*')):
for path in sorted(Path.cwd().glob('*_triangle*')):
    print('--')
    print(path.name)
    m = re.search(TIMESTAMP_RE, path.name)
    old_ts = datetime(*[int(g) for g in m.groups()])
    new_ts = old_ts + CORRECTION
    if "T" in path.name:
	new_name = path.name[:m.start()] + new_ts.strftime("%Y-%m-%dT%H.%M.%S") \
                + path.name[m.end():]
    else:
        new_name = path.name[:m.start()] + new_ts.strftime("%Y-%m-%d-%H-%M-%S") \
                + path.name[m.end():]
    print(new_name)
    new_file = path.parent / new_name
    path.rename(new_file)
    
