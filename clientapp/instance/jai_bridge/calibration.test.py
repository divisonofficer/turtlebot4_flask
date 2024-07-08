from calibration import Calibration

calibration = Calibration()


def test_calibration_load():
    STORAGE_ID = 4
    calibration.CHESS_CELL_WIDTH = 19.1
    calibration.CHESS_SHAPE = (13, 9)
    calibration.update_objp()
    calibration.load_calibration(STORAGE_ID)


if __name__ == "__main__":

    test_calibration_load()
