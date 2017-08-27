#!/usr/bin/python3
"""Visualise results on a laptop after calibrating the MinIMU compass.
"""
import os
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.optimize import leastsq
from subprocess import run
from tempfile import TemporaryDirectory

def add_pitch_roll(df):
    df['pitch_r'] = np.arctan2(df.acc_x, np.sqrt(df.acc_y**2 + df.acc_z**2))
    df['pitch'] = np.degrees(df['pitch_r'])
    df['roll_r'] = np.arctan2(-df.acc_y, -df.acc_z)
    df['roll'] = np.degrees(df['roll_r'])

def compensate_mag_y(p, df):
    return (df.mag_x * np.sin(df.roll_r) * np.sin(df.pitch_r)) \
        + (df.mag_y * np.cos(df.roll_r)) \
        - (((df.mag_z - p[0]) / p[1])* np.sin(df.roll_r) * np.cos(df.pitch_r))

def optimize_roll_compensation(df):
    # Take our correct y field as the points where we're within 3 degrees of level.
    y_flat = df[(-3 < df.roll) & (df.roll < +3)].mag_y.mean()

    def mag_y_comp_residuals(p):
        return compensate_mag_y(p, df) - y_flat

    res, ier = leastsq(mag_y_comp_residuals, (1, 1))
    # According to the docs, an integer code between 1 and 4 (inclusive) indicates
    # success.
    assert 1 <= ier <= 4
    return res, y_flat

def make_plots(level, roll):
    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(14, 8))
    
    # x/y field from level calibration
    level.plot(x='mag_x', y='mag_y',
                xlim=(level.mag_x.min() - 100, level.mag_x.max() + 100),
                ax=axes[0, 0], legend=False)
    axes[0, 0].set_ylabel('mag y')

    # Pitch and roll from level calibration
    level.plot(y='pitch', ax=axes[0, 1])
    level.plot(y='roll', ax=axes[0, 1])
    axes[0, 1].hlines(0, 0, len(level), linestyles='dotted')
    axes[0, 1].set_ylim(-30, 30)
    
    # z field from level calibration
    level.plot(y='mag_z', ax=axes[0, 2], legend=False)
    axes[0, 2].set_title('mag z')

    # Pitch and roll from roll calibration
    roll.plot(y='pitch', ax=axes[1, 0])
    roll.plot(y='roll', ax=axes[1, 0])
    axes[1, 0].hlines(0, 0, len(roll), linestyles='dotted')
    axes[1, 0].set_ylim(-60, 60)
    
    # Mag y against roll
    roll.plot(x='roll', y='mag_y', ax=axes[1, 2])
    param, y_flat = optimize_roll_compensation(roll)
    roll['mag_y_compensated'] = compensate_mag_y(param, roll)
    roll.plot(x='roll', y='mag_y_compensated', ax=axes[1, 2], xlim=(-60, 60))
    axes[1, 2].vlines(0, roll.mag_y.min(), roll.mag_y.max(), linestyles='dotted')
    axes[1, 2].hlines(y_flat, -60, 60, linestyles='dotted')

    fig.text(0.93, 0.7, 'Waltz', rotation=270, size='x-large')
    fig.text(0.93, 0.3, 'Rock', rotation=270, size='x-large')

    return fig, axes

pi_addr = "pi@{}".format(os.environ.get('SAIL_PI_IP', '192.168.12.1'))
print('Getting data from', pi_addr)
pi_folder = pi_addr + ":sailing-robot/utilities/"

with TemporaryDirectory() as td:
    td = Path(td)
    run(['scp', pi_folder + 'latest_calibration_time', td], check=True)
    with (td / 'latest_calibration_time').open() as f:
        ts = f.read().strip()

    level_file = td / 'calibration_level_{}.csv'.format(ts)
    roll_file = td / 'calibration_roll_{}.csv'.format(ts)    
    print("Fetching files with timestamp", ts)
    run(['scp', pi_folder + 'calibration_*_{}.csv'.format(ts), td], check=True)
    #print("Fetching", roll_file)
    #run(['scp', pi_folder + roll_file.name, td], check=True)

    level = pd.read_csv(level_file)
    add_pitch_roll(level)
    roll = pd.read_csv(roll_file)
    add_pitch_roll(roll)

    fig, axes = make_plots(level, roll)
    plt.show()
