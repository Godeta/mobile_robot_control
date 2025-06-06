#!/usr/bin/env pybricks-micropython

import time
import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
fork_motor = Motor(Port.D)

# Sensors
gyro_sensor = GyroSensor(Port.S3)

# Set up initial odometry and drive base
class OdometrieRobot:
    def __init__(self, rayon_roue, entraxe):
        self.rayon_roue = rayon_roue        # en cm
        self.entraxe = entraxe              # distance entre roues en cm
        self.x = 25.5                        # position en X (cm)
        self.y = 4.8                        # position en Y (cm)
        self.theta = 0.0                   # orientation (radian)
        self.distance_droite_precedent = 0.0
        self.distance_gauche_precedent = 0.0

    def mettre_a_jour(self):
        # distances in cm
        distance_droite = (right_motor.angle()*-1 / 360) * self.rayon_roue * 2 * math.pi
        distance_gauche = (left_motor.angle()*-1 / 360) * self.rayon_roue * 2 * math.pi

        delta_droite = distance_droite - self.distance_droite_precedent
        delta_gauche = distance_gauche - self.distance_gauche_precedent

        self.distance_droite_precedent = distance_droite
        self.distance_gauche_precedent = distance_gauche

        delta_distance = (delta_droite + delta_gauche) / 2

        self.theta = (gyro_sensor.angle() * (-math.pi / 180) + math.pi) % (2 * math.pi) - math.pi

        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)

    def get_position(self):
        return self.x, self.y, self.theta

def vitesse_roue(V, W, L, R):
    Vd = (V + L * W) / R
    Vg = (V - L * W) / R
    return Vd, Vg

def ControlLaw(Xr, Yr, X, Y, theta, Kp, KIp, Ka, KIa, dt, integral_p, integral_alpha):
    p = math.sqrt((Xr - X)**2 + (Yr - Y)**2)
    alpha = AngleWrap(math.atan2((Yr - Y), (Xr - X)) - theta)

    integral_p += p * dt
    integral_alpha += alpha * dt

    V = Kp * p + KIp * integral_p
    W = Ka * alpha + KIa * integral_alpha

    return V, W, integral_p, integral_alpha

def AngleWrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def current_milli_time():
    return round(time.time() * 1000)

def scaleSpeed(Vg_linear, Vd_linear, R):
    # Convert linear cm/s to rotational speed in deg/s
    Vg_deg = (Vg_linear * 360) / (2 * math.pi * R)
    Vd_deg = (Vd_linear * 360) / (2 * math.pi * R)

    max_speed = 1050  # deg/s for EV3 Large Motor

    max_abs = max(abs(Vg_deg), abs(Vd_deg))
    if max_abs > max_speed:
        scale = max_abs / max_speed
        Vg_deg /= scale
        Vd_deg /= scale

    return Vg_deg, Vd_deg


if __name__ == "__main__":
    # Coordonné X,Y en cm
    Zone_deche_app = [15,112]
    Zone_deche_entre = [38,128]
    Zone_deche_sorti = [55,129]
    Zone_trie = [122,100]
    Traj = [Zone_deche_app, Zone_deche_entre, Zone_deche_sorti, Zone_trie]

    step = 0

    coef=1.5
    # Gains
    Kp = 0.5 * coef
    KIp = 0.05 * coef
    Ka = 1.5 * coef
    KIa = 0

    dt = 0.1  # seconds

    integral_p = 0.0
    integral_alpha = 0.0

    robot = OdometrieRobot(rayon_roue=24, entraxe=12.5)

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    fork_motor.reset_angle(0)
    gyro_sensor.reset_angle(-90)

    regul_interval = current_milli_time()

    while True:
        if current_milli_time() >= regul_interval:
            robot.mettre_a_jour()
            x, y, theta = robot.get_position()
            Coordonne = Traj[step]
            Xr = Coordonne[0]
            Yr = Coordonne[1]
            V, W, integral_p, integral_alpha = ControlLaw(Xr, Yr, x, y, theta, Kp, KIp, Ka, KIa, dt, integral_p, integral_alpha)
            Vd, Vg = vitesse_roue(V, W, robot.entraxe, robot.rayon_roue)

            scaleVg, scaleVd = scaleSpeed(Vg, Vd, robot.rayon_roue)

            # Convert linear cm/s to motor angular speed (deg/s)
            left_motor.run(int(scaleVg)*-1)
            right_motor.run(int(scaleVd)*-1)

            # Print for debugging
            ev3.screen.clear()
            ev3.screen.print("x: {:.1f}".format(x))
            ev3.screen.print("y: {:.1f}".format(y))
            ev3.screen.print("θ: {:.2f}".format(math.degrees(theta)))

            # Check if robot has reached the goal (within 2 cm)
            if math.sqrt((Xr - x) ** 2 + (Yr - y) ** 2) < 2:
                step += 1
            
            if step >= 4:
                left_motor.brake()
                right_motor.brake()
                fork_motor.run_angle(500, 60)  # Adjust speed and angle as needed
                break  # Exit the loop after lifting the fork


            regul_interval = current_milli_time() + int(dt * 1000)
