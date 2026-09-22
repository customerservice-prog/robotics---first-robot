from app.robot import mix_drive


def test_forward():
    assert mix_drive(1, 0, 65) == (65, 65)


def test_reverse():
    assert mix_drive(-1, 0, 65) == (-65, -65)


def test_turn_in_place():
    assert mix_drive(0, 1, 65) == (65, -65)


def test_mix_is_bounded():
    left, right = mix_drive(1, 1, 70)
    assert abs(left) <= 70
    assert abs(right) <= 70
