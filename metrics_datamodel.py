# metrics_datamodel.py

import math
import random # Add random import here

class MetricsData:
    """A data class to hold and parse telemetry data from JULES."""
    
    # ... __init__ method is the same ...
    def __init__(self):
        self.t = 0.0
        self.cmdPower = 0.0
        self.velIPS = 0.0
        self.headingDeg = 0.0
        self.batteryV = 0.0
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yawRate = 0.0
        self.pitchRate = 0.0
        self.rollRate = 0.0

    # ... update_from_dict method is the same ...
    def update_from_dict(self, json_dict):
        """Updates the object's attributes from a parsed JSON dictionary."""
        self.t = json_dict.get('ts', self.t)
        self.cmdPower = json_dict.get('cmd', self.cmdPower)
        self.velIPS = json_dict.get('vel_ips', self.velIPS)
        self.headingDeg = json_dict.get('heading_deg', self.headingDeg)
        self.batteryV = json_dict.get('battery_v', self.batteryV)
        self.x = json_dict.get('x', self.x)
        self.y = json_dict.get('y', self.y)
        self.heading = json_dict.get('heading', self.heading)
        self.pitch = json_dict.get('pitch', self.pitch)
        self.roll = json_dict.get('roll', self.roll)
        self.yawRate = json_dict.get('yawRate', self.yawRate)
        self.pitchRate = json_dict.get('pitchRate', self.pitchRate)
        self.rollRate = json_dict.get('rollRate', self.rollRate)

    # ... update_for_fake_generator method is the same ...
    def update_for_fake_generator(self, elapsed_time):
        """Updates attributes with fake data for testing."""
        self.t = elapsed_time
        radius = 24
        speed = 12
        angular_velocity = speed / radius
        angle = angular_velocity * elapsed_time
        self.x = radius * math.cos(angle)
        self.y = radius * math.sin(angle)
        self.heading = (angle + math.pi / 2) % (2 * math.pi)
        self.velIPS = speed + (random.random() - 0.5)
        self.headingDeg = math.degrees(self.heading)
        self.batteryV = 12.5 - elapsed_time * 0.01
        self.cmdPower = 0.5 + 0.1 * math.sin(elapsed_time)
        self.pitch = 1.0 + (random.random() - 0.5) * 0.5
        self.roll = -0.5 + (random.random() - 0.5) * 0.5
        self.yawRate = math.degrees(angular_velocity) + (random.random() - 0.5)
        self.pitchRate = (random.random() - 0.5) * 0.2
        self.rollRate = (random.random() - 0.5) * 0.2

    def to_dict(self):
        """Converts the object's attributes to a dictionary."""
        return {
            'ts': self.t,
            'cmd': self.cmdPower,
            'vel_ips': self.velIPS,
            'heading_deg': self.headingDeg,
            'battery_v': self.batteryV,
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'pitch': self.pitch,
            'roll': self.roll,
            'yawRate': self.yawRate,
            'pitchRate': self.pitchRate,
            'rollRate': self.rollRate
        }