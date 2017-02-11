#!/usr/bin/python
from __future__ import print_function
from datetime import datetime
import io
from itertools import groupby
from jinja2 import Environment, FileSystemLoader
import os
from os.path import dirname, realpath

ISO8601 = "%Y-%m-%dT%H:%M:%SZ"

utils_dir = dirname(realpath(__file__))
repo_dir = dirname(utils_dir)
data_dir = os.path.join(repo_dir, 'recorded_data')

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

class GPSTrace(DataFile):
    file_type = GPS_TRACE

class ParamDump(DataFile):
    file_type = PARAM_DUMP

def parse_filename(name):
    """Parse the name of a file in the recorded_data directory.
    
    Returns an instance of one of the DataFile subclasses above.
    """
    if name.endswith('.bag'):
        test, _, timestamp = name[:-len('.bag')].rpartition('_')
        dt = datetime.strptime(timestamp, '%Y-%m-%d-%H-%M-%S')
        return Rosbag(name, test, dt)
    if name.startswith('params-dump_') and name.endswith('.json'):
        s = name[len('params-dump_'):-len('.json')]
        if '_' in s:
            test, _, timestamp = s.rpartition('_')
        else:
            test, timestamp = '', s
        dt = datetime.strptime(timestamp, '%Y-%m-%d-T%H-%M-%S')
        return ParamDump(name, test, dt)
    if name.startswith('gps-trace') and name.endswith('.csv'):
        s = name[len('gps-trace_'):-len('.csv')]
        if '_' in s:
            test, _, timestamp = s.rpartition('_')
        else:
            test, timestamp = '', s
        dt = datetime.strptime(timestamp, '%Y-%m-%dT%H.%M.%S')
        return GPSTrace(name, test, dt)

class FileGroup(object):
    def __init__(self, rosbag):
        self.rosbag = rosbag
        self.others = []
    
    def __iter__(self):
        yield self.rosbag
        for f in self.others:
            yield f
    
    def __len__(self):
        return len(self.others) + 1

def scan_recorded_data_files():
    by_type = {ROSBAG: [], GPS_TRACE: [], PARAM_DUMP: []}
    for name in os.listdir(data_dir):
        data_file = parse_filename(name)
        if data_file is None:
            continue
        
        by_type[data_file.file_type].append(data_file)
    
    # Use rosbags as anchors, and group related files with start times within
    # 10 seconds.
    groups = []
    for bag in by_type[ROSBAG]:
        group = FileGroup(bag)
        for file_type in [GPS_TRACE, PARAM_DUMP]:
            for file in by_type[file_type]:
                if abs((file.start - bag.start).total_seconds()) < 10:
                    group.others.append(file)
        groups.append(group)
    
    groups.sort(key=lambda g: g.rosbag.start)
    return groups

def generate_html(data_groups):
    jinja_env = Environment(loader=FileSystemLoader(utils_dir),
                            autoescape=True)
    template = jinja_env.get_template('data_index.tpl')
    
    data_by_days = groupby(data_groups, key=lambda g: g.rosbag.start.date())
    output_file = os.path.join(data_dir, 'index.html')
    with io.open(output_file, 'w', encoding='utf-8') as f:
        template.stream(days=data_by_days).dump(f)
    print('Index written to', output_file)
    
if __name__ == '__main__':
    groups = scan_recorded_data_files()
    generate_html(groups)
