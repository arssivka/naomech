import sys

import time
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *

from robot import Robot
from walker import Walker

stance = [310.0, 14.5, 100.0, 0.05, 0.0, 0.1]
step = [0.75, 0.45, 10.0, 0.0, 90.0, -50.0, 70.0, 0.35, 70.0, 70.0, 0.35, 1.0]
zmp = [0.0, 0.9, 30.0, 30.0, 0.01, 6.6]
hack = [0.05, 0.05]
sensor = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
stiff = [0.85, 0.6, 0.4, 0.3, 0.2, 0.2]
odo = [1.0, 1.0, 1.3]
arm = [0.3]

def create_configurator(prnt, min, max, step, name, array, index, func=None):
    spin = QDoubleSpinBox(prnt)
    spin.setRange(min, max)
    spin.setSingleStep(step)
    spin.setValue(array[index])

    lcd = QLCDNumber(prnt)
    lcd.setDigitCount(5)
    lcd.display(array[index])

    spin.valueChanged.connect(lcd.display)
    if func:
        def set_value(value):
            array[index] = func(value)
        spin.valueChanged.connect(set_value)
    else:
        def set_value(value):
            array[index] = value
        spin.valueChanged.connect(set_value)

    layout = QHBoxLayout()
    layout.addWidget(QLabel(name, prnt))
    layout.addWidget(spin)
    layout.addWidget(lcd)
    return layout

def create_group(name, array, names, func=None):
    if len(array) != len(names):
        raise RuntimeError("%s :Incorrect number of arguments" % name)
    gb = QGroupBox(name)
    layout = QVBoxLayout()
    gb.setLayout(layout)
    for index, name in zip(range(len(array)), names):
        layout.addLayout(create_configurator(gb, -10000.0, 10000.0, 1.5, name, array, index, func))
    layout.addWidget(gb)
    return gb


def send():
    send.butt.setEnabled(False)
    robot = Robot("192.168.0.14", "5469")
    walk = Walker(robot)
    robot.locomotion.gait(stance, step, zmp, hack, sensor, stiff, odo, arm)
    walk.linear_go_to(300.0, 0.0, 100.0)
    time.sleep(1.0)
    while not walk.is_done():
        time.sleep(1.0)
    walk.stop()
    send.butt.setEnabled(True)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    super_main = QWidget()
    main = QScrollArea()
    main.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
    main.setWindowTitle("Gait configurator")
    main.setWidgetResizable(False)
    butt = QPushButton("Send")
    send.butt = butt
    butt.clicked.connect(send)
    main_layout = QVBoxLayout()
    for group, (names, array) in {"stance": (("com_heght", "x_off", "y_separation", "angle_y", "foot_angle", "transition_time"), stance),
                                  "step": (("step_time", "dblSupFrac", "step_height", "lift_angle", "max_forward_vel_x", "max_backward_vel_x",
                                            "max_vel_y", "max_vel_t", "max_acc_x", "max_acc_y", "max_acc_t", "walking_gait"), step),
                                  "ZMP": (("foot_center", "zmp_static_perc", "l_zmp_off", "r_zmp_off", "strate_zmp_off", "turn_zmp_off"), zmp),
                                  "hack": (("hip_hack_l", "hip_hack_r"), hack),
                                  "sensor": (("observer_sacle", "GX", "GY", "KX", "KY", "MAXVELX", "MAXVELY", "angle_xy_scale"), sensor),
                                  "stiff": (("hip", "knee", "ap", "ar", "arm", "arm_pitch"), stiff),
                                  "odo": (("x", "y", "theta"), odo),
                                  "arm": (("arm_aplitude",), arm)}.items():
        main_layout.addWidget(create_group(group, array, names))
    main_layout.addStretch(1)
    wgt = QWidget()
    wgt.setLayout(main_layout)
    main.setWidget(wgt)
    layout = QVBoxLayout()
    layout.addWidget(main)
    layout.addWidget(butt)
    super_main.setLayout(layout)
    super_main.show()
    sys.exit(app.exec_())