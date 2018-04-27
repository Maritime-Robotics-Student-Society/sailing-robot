class Tacking(object):
    # This doesn't yet have any state, but it may in the future
    def calculate_sail_and_rudder(self, state):
        if state in ('switch_to_port_tack', ):
            # Turning right
            return 0, -90
        elif state in ('switch_to_stbd_tack', ):
            # Turning left
            return 0, 90
        else:
            # Normal sailing - it should use sail and rudder angles from elsewhere
            return 0, 0
