#!/usr/bin/python3
import os
from matplotlib import pyplot as plt
import pandas as pd
from subprocess import run
from tempfile import TemporaryDirectory

pi_addr = "pi@{}".format(os.environ.get('SAIL_PI_IP', '192.168.42.1'))
print('Getting data from', pi_addr)
pi_folder = pi_addr + ":sailing-robot/utilities/"

def make_plots(level, roll):
    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(14, 8))
    
    # x/y field from level calibration
    level.plot(x='mag_x', y='mag_y', xlim=(df.mag_x.min() - 100, df.mag_x.max() + 100),
                ax=axes[0, 0], legend=False)
    axes[0, 0].set_ylabel('mag y')

    # Pitch and roll from level calibration
    df.plot(y='pitch', ax=axes[0, 1])
    df.plot(y='roll', ax=axes[0, 1])
    axes[0, 1].hlines(0, 0, len(df), linestyles='dotted')
    axes[0, 1].set_ylim(-30, 30)
    
    # z field from level calibration
    df.plot(y='mag_z', ax=axes[0, 2], legend=False)
    axes[0, 2].set_title('mag z')

    # 

    return fig, axes

with TemporaryDirectory() as td:
    td = Path(td)
    run(['scp', pi_folder + 'latest_calibration_time', td], check=True)
    with (td / 'latest_calibration_time').open() as f:
        ts = f.read().strip()

    level_file = td / 'calibration_level_{}.csv'.format(ts)
    roll_file = td / 'calibration_roll_{}.csv'.format(ts)    
    print("Fetching", level_file)
    run(['scp', pi_folder + level_file.name, td], check=True)
    print("Fetching", roll_file)
    run(['scp', pi_folder + roll_file.name, td], check=True)

    level = pd.read_csv(level_file)
    roll = pd.read_csv(roll_file)

    fig, axes = make_plots(level, roll)
    plt.show()
