from pytest import approx
from sailing_robot.sail_table import SailTable, SailData

SAMPLE_SAIL_TABLE = {
    '0': 0,
    '30': 0,
    '90': 0.5,
    '180': 0.9,
}

def test_sail_table():
    st = SailTable(SAMPLE_SAIL_TABLE)
    assert st.interpolate_sail_setting(20) == approx(0)
    assert st.interpolate_sail_setting(60) == approx(0.25)
    assert st.interpolate_sail_setting(90) == approx(0.5)
    assert st.interpolate_sail_setting(135) == approx(0.7)
    assert st.interpolate_sail_setting(190) == approx(0.9)

def test_sail_data():
    st = SailTable(SAMPLE_SAIL_TABLE)
    sd = SailData(st)
    sd.wind_direction_apparent = 90
    assert sd.calculate_sheet_setting() == approx(0.5)
