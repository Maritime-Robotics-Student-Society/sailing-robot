from nose.tools import assert_equal, assert_almost_equal
from sailing_robot.sail_table import SailTable

def test_sail_table():
    st = SailTable({
        '0': 0,
        '30': 0,
        '90': 0.5,
        '180': 0.9,
    })
    assert_almost_equal(st.interpolate_sail_setting(20), 0)
    assert_almost_equal(st.interpolate_sail_setting(60), 0.25)
    assert_almost_equal(st.interpolate_sail_setting(90), 0.5)
    assert_almost_equal(st.interpolate_sail_setting(135), 0.7)
    assert_almost_equal(st.interpolate_sail_setting(190), 0.9)
