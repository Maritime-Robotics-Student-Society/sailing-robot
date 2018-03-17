"""Intepolate a sail setting from a table of wind_direction -> sail_setting
"""

from __future__ import division

class SailTable(object):
    def __init__(self, table_dict):
        """table_dict should be a mapping from wind direction (in degrees) to
        sail setting (0=fully in to 1=fully out).
        
        Because of the limitations of the YAML parameter files, the keys may be
        in strings.
        """
        self.table = sorted([(int(k), v) for k,v in table_dict.items()])
    
    def interpolate_sail_setting(self, wind_direction):
        """Turn a wind angle in degrees into a sail setting (0 to 1).
        
        Wind angle should be between 0 and 180, so e.g. 315 should be normalised
        to 45 before calling this.
        """
        last_wind_direction, last_sail_setting = self.table[0]
        
        # Loop through the table to find which entries the current wind
        # direction is between.
        for next_wind_direction, next_sail_setting in self.table[1:]:
            if next_wind_direction > wind_direction:
                # It's in the interval (last_wind_direction, next_wind_direction)
                if next_wind_direction == last_wind_direction:
                    delta = 0   # Avoid division by 0
                else:
                    # Where are we within the interval?
                    delta = (wind_direction - last_wind_direction) \
                            / (next_wind_direction - last_wind_direction)
                
                # Interpolated value
                return last_sail_setting + \
                        delta * (next_sail_setting - last_sail_setting)
            
            # Haven't found the interval yet; update the lower bound for the
            # next loop.
            last_wind_direction, last_sail_setting = \
                next_wind_direction, next_sail_setting
        
        # We ran past the end of the table
        return last_sail_setting

class SailData(object):
    def __init__(self, sail_table):
        self.wind_direction_apparent = 0
        self.sailing_state = 'normal'
        self.sail_table = sail_table

    def update_wind(self, msg):
        self.wind_direction_apparent = msg.data

    def update_sailing_state(self, msg):
        self.sailing_state = msg.data

    def calculate_sheet_setting(self):
        windDirection = self.wind_direction_apparent
        if windDirection > 180:
            windDirection = 360 - windDirection

        return self.sail_table.interpolate_sail_setting(windDirection)
