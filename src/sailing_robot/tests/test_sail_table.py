from nose.tools import assert_equal, assert_almost_equal
from sailing_robot.sail_table import SailTable, SailData

SAMPLE_SAIL_TABLE = {
    '0': 0,
    '30': 0,
    '90': 0.5,
    '180': 0.9,
}

def test_sail_table():
    st = SailTable(SAMPLE_SAIL_TABLE)
    assert_almost_equal(st.interpolate_sail_setting(20), 0)
    assert_almost_equal(st.interpolate_sail_setting(60), 0.25)
    assert_almost_equal(st.interpolate_sail_setting(90), 0.5)
    assert_almost_equal(st.interpolate_sail_setting(135), 0.7)
    assert_almost_equal(st.interpolate_sail_setting(190), 0.9)

def test_sail_data():
    st = SailTable(SAMPLE_SAIL_TABLE)
    sd = SailData(st)
    sd.wind_direction_apparent = 90
    assert_almost_equal(sd.calculate_sheet_setting(), 0.5)
