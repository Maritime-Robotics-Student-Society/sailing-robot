#!/usr/bin/python
from __future__ import print_function

import argparse
from datetime import datetime, timedelta
from glob import glob
import io
from itertools import groupby
from jinja2 import Environment, FileSystemLoader
import json
import os
from os.path import dirname, realpath
import re
import sys

ISO8601 = "%Y-%m-%dT%H:%M:%SZ"

utils_dir = dirname(realpath(__file__))
repo_dir = dirname(utils_dir)
data_dir = os.path.join(repo_dir, 'recorded_data')
notes_dir = os.path.join(data_dir, 'notes')

class DataFile(object):
    def __init__(self, filename, test_name, start):
        self.filename = filename
        self.test_name = test_name
        self.start = start

    @property
    def path(self):
        return os.path.join(data_dir, self.filename)
    
    def __repr__(self):
        return '{}({!r}, {!r}, {!r})'.format(
            type(self).__name__, self.filename, self.test_name, self.start
        )

ROSBAG = 'rosbag'
GPS_TRACE = 'gps_trace'
PARAM_DUMP = 'param_dump'

class Rosbag(DataFile):
    file_type = ROSBAG
    n_messages = None
    duration = None
    topic_list = None
    
    def read_info(self):
        """Get some metadata about this rosbag.
        
        Uses a JSON metadata file as a cache."""
        md_file = self.path + '.metadata'
        if os.path.exists(md_file):
            with open(md_file) as f:
                d = json.load(f)
        else:
            d = self._read_info()
            with open(md_file, 'w') as f:
                json.dump(d, f)

        self.n_messages = d['n_messages']
        self.duration = d['duration']
        self.topic_list = d['topic_list']
    
    def _read_info(self):
        """Read some metadata from inside this rosbag."""
        from rosbag import Bag, ROSBagUnindexedException, ROSBagException
        try:
            b = Bag(self.path)
        except ROSBagUnindexedException:
            b = Bag(self.path, allow_unindexed=True)
            print('Reindexing', self.filename)
            b.reindex()

        try:
            duration = b.get_end_time() - b.get_start_time()
        except ROSBagException:
            duration = 0
        return {
            'n_messages': b.get_message_count(),
            'duration': duration,
            'topic_list': b.get_type_and_topic_info()[1].keys()
        }

    @property
    def end(self):
        return self.start + timedelta(seconds=self.duration)

class GPSTrace(DataFile):
    file_type = GPS_TRACE

class ParamDump(DataFile):
    file_type = PARAM_DUMP

# Different bits of code have made timestamp filenames with different patterns
TIMESTAMP_RE = r'(\d{4})-(\d{2})-(\d{2})-?T?(\d{2})[-.](\d{2})[-.](\d{2})Z?$'
def parse_timestamp(filename_ts):
    m = re.match(TIMESTAMP_RE, filename_ts)
    norm_ts = m.expand(r'\1-\2-\3T\4.\5.\6')
    return datetime.strptime(norm_ts, '%Y-%m-%dT%H.%M.%S')

def parse_filename(name):
    """Parse the name of a file in the recorded_data directory.
    
    Returns an instance of one of the DataFile subclasses above.
    """
    if name.endswith('.bag'):
        test, _, timestamp = name[:-len('.bag')].rpartition('_')
        dt = parse_timestamp(timestamp)
        return Rosbag(name, test, dt)
    if name.startswith('params-dump_') and name.endswith('.json'):
        s = name[len('params-dump_'):-len('.json')]
        if '_' in s:
            test, _, timestamp = s.rpartition('_')
        else:
            test, timestamp = '', s
        dt = parse_timestamp(timestamp)
        return ParamDump(name, test, dt)
    if name.startswith('gps-trace') and name.endswith('.csv'):
        s = name[len('gps-trace_'):-len('.csv')]
        if '_' in s:
            test, _, timestamp = s.rpartition('_')
        else:
            test, timestamp = '', s
        dt = parse_timestamp(timestamp)
        return GPSTrace(name, test, dt)

class FileGroup(object):
    prop_size = 0
    def __init__(self, rosbag):
        self.rosbag = rosbag
        self.others = []
        self.notes = []
    
    def __iter__(self):
        yield self.rosbag
        for f in self.others:
            yield f
    
    def __len__(self):
        return len(self.others) + 1

def load_notes():
    notes = []
    for path in glob(os.path.join(notes_dir, '*.json')):
        filename_ts = os.path.splitext(os.path.basename(path))[0]
        ts = parse_timestamp(filename_ts)
        with open(path, 'r') as f:
            note = json.load(f)
        note['timestamp'] = ts
        notes.append(note)
    
    return sorted(notes, key=lambda n: n['timestamp'])

def scan_recorded_data_files():
    by_type = {ROSBAG: [], GPS_TRACE: [], PARAM_DUMP: []}
    for name in os.listdir(data_dir):
        data_file = parse_filename(name)
        if data_file is None:
            continue
        
        by_type[data_file.file_type].append(data_file)
    
    notes = load_notes()
    
    # Use rosbags as anchors, and group related files with start times within
    # 10 seconds.
    groups = []
    for bag in by_type[ROSBAG]:
        bag.read_info()
        group = FileGroup(bag)
        for file_type in [GPS_TRACE, PARAM_DUMP]:
            for file in by_type[file_type]:
                if abs((file.start - bag.start).total_seconds()) < 10:
                    group.others.append(file)
        
        # Add notes which were written during this run, or within a minute of it
        for note in notes:
            if (bag.start - timedelta(seconds=60)) < note['timestamp'] \
                    < (bag.end + timedelta(seconds=60)):
                group.notes.append(note)

        groups.append(group)
    
    max_duration = max(g.rosbag.duration for g in groups)
    for g in groups:
        g.prop_size = int(100 * (g.rosbag.duration / max_duration))
    
    groups.sort(key=lambda g: g.rosbag.start, reverse=True)
    return groups

def seconds_to_mins(s):
    return int(s/60)

def generate_html(data_groups):
    jinja_env = Environment(loader=FileSystemLoader(utils_dir),
                            autoescape=True)
    jinja_env.filters['seconds_to_mins'] = seconds_to_mins
    template = jinja_env.get_template('data_index.tpl')
    
    data_by_days = groupby(data_groups, key=lambda g: g.rosbag.start.date())
    output_file = os.path.join(data_dir, 'index.html')
    with io.open(output_file, 'w', encoding='utf-8') as f:
        template.stream(days=data_by_days).dump(f)
    print('Index written to', output_file)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--docker', action='store_true',
                    help='Launch a docker container with ROS to run this script')
    args = ap.parse_args()
    
    if args.docker:
        from subprocess import call
        rc = call(['docker', 'run', '-v', repo_dir+':/home/pi/sailing-robot',
                '--rm', 'jamak9/sailing-robot:indigo', 'python', '/home/pi/sailing-robot/utilities/index_recorded_data.py'])
        sys.exit(rc)
    
    groups = scan_recorded_data_files()
    generate_html(groups)


if __name__ == '__main__':
    main()
