from arm_gui.layouts import APP, GAZEBO, RVIZ, Rect, WindowPlacer, fit_to_screens, parse_wmctrl, preset_layout

LAPTOP = Rect(0, 0, 1920, 1050)
MONITOR = Rect(1920, 0, 2560, 1400)


def test_laptop_preset_tiles_one_screen_without_overlap():
    layout = preset_layout("laptop", [LAPTOP])
    assert set(layout) == {APP, GAZEBO, RVIZ}
    assert layout[GAZEBO].y == layout[RVIZ].y == 0
    assert layout[GAZEBO].w + layout[RVIZ].w == LAPTOP.w
    assert layout[APP].y == layout[GAZEBO].h and layout[APP].y + layout[APP].h == LAPTOP.h


def test_two_screens_puts_app_alone_and_sim_views_on_the_second_screen():
    layout = preset_layout("two_screens", [LAPTOP, MONITOR])
    assert layout[APP] == LAPTOP
    assert MONITOR.contains(layout[GAZEBO].center) and MONITOR.contains(layout[RVIZ].center)


def test_two_screens_falls_back_to_laptop_with_one_screen():
    assert preset_layout("two_screens", [LAPTOP]) == preset_layout("laptop", [LAPTOP])


def test_no_gazebo_preset_has_no_gazebo_window():
    assert set(preset_layout("no_gazebo", [LAPTOP])) == {APP, RVIZ}


def test_window_left_on_an_unplugged_monitor_comes_back_to_the_primary_screen():
    lost = Rect(2500, 200, 1200, 800)
    fitted = fit_to_screens(lost, [LAPTOP])
    assert LAPTOP.contains(fitted.center)
    assert (fitted.w, fitted.h) == (1200, 800)


def test_window_larger_than_its_screen_is_shrunk_and_kept_inside():
    fitted = fit_to_screens(Rect(100, 100, 5000, 3000), [LAPTOP])
    assert fitted == LAPTOP


def test_visible_window_is_kept_where_it_is():
    r = Rect(2000, 100, 800, 600)
    assert fit_to_screens(r, [LAPTOP, MONITOR]) == r


WMCTRL = """0x03000007  0 1920 0    1280 1400 host Gazebo Sim
0x04200003  0 3200 0    1280 1400 host display.rviz* - RViz
0x05000001  0 0    0    1920 1050 host Arm Studio — ρομποτικός βραχίονας
"""


def test_parse_wmctrl_finds_gazebo_and_rviz():
    found = parse_wmctrl(WMCTRL)
    assert found[GAZEBO] == ("0x03000007", Rect(1920, 0, 1280, 1400))
    assert found[RVIZ][0] == "0x04200003"
    assert APP not in found


def test_placer_issues_wmctrl_move():
    calls = []

    class Result:
        returncode = 0
        stdout = WMCTRL

    def run(cmd, **_kw):
        calls.append(cmd)
        return Result()

    placer = WindowPlacer(run=run)
    if not placer.available:  # wmctrl missing on this machine: only the parsing path is meaningful
        return
    assert placer.move("0x1", Rect(1, 2, 3, 4))
    assert calls[-1] == ["wmctrl", "-i", "-r", "0x1", "-e", "0,1,2,3,4"]
