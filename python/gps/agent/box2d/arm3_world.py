""" This file defines an environment for the Box2D 2 Link Arm simulator. """
import Box2D as b2
import numpy as np
from framework import Framework
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES

class Arm3World(Framework):
    """ This class defines a 3 Link Arm and its environment."""
    name = "3 Link Arm"
    def __init__(self, x0, target):
        super(Arm3World, self).__init__()

        self.world.gravity = (0.0, 0.0)

        fixture_length = 5.0
        self.x0 = x0

        rectangle_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(.5, fixture_length)),
            density=.5,
            friction=1,
        )
        square_fixture = b2.b2FixtureDef(
            shape=b2.b2PolygonShape(box=(1, 1)),
            density=100.0,
            friction=1,
        )
        self.base = self.world.CreateBody(
            position=(0, 15),
            fixtures=square_fixture,
        )

        self.body1 = self.world.CreateDynamicBody(
            position=(0, 2),
            fixtures=rectangle_fixture,
            angle=b2.b2_pi,
        )
        self.body2 = self.world.CreateDynamicBody(
            fixtures=rectangle_fixture,
            position=(0, 2),
            angle=b2.b2_pi,
        )
        self.body3 = self.world.CreateDynamicBody(
            fixtures=rectangle_fixture,
            position=(0, 2),
            angle=b2.b2_pi,
        )


        self.target1 = self.world.CreateDynamicBody(
            fixtures=rectangle_fixture,
            position=(0, 0),
            angle=b2.b2_pi,
        )
        self.target2 = self.world.CreateDynamicBody(
            fixtures=rectangle_fixture,
            position=(0, 0),
            angle=b2.b2_pi,
        )
        self.target3 = self.world.CreateDynamicBody(
            fixtures=rectangle_fixture,
            position=(0, 0),
            angle=b2.b2_pi,
        )

        self.joint1 = self.world.CreateRevoluteJoint(
            bodyA=self.base,
            bodyB=self.body1,
            localAnchorA=(0, 0),
            localAnchorB=(0, fixture_length),
            enableMotor=True,
            maxMotorTorque=400,
            enableLimit=False,
        )

        self.joint2 = self.world.CreateRevoluteJoint(
            bodyA=self.body1,
            bodyB=self.body2,
            localAnchorA=(0, -(fixture_length - 0.5)),
            localAnchorB=(0, fixture_length - 0.5),
            enableMotor=True,
            maxMotorTorque=400,
            enableLimit=False,
        )

        self.joint3 = self.world.CreateRevoluteJoint(
            bodyA=self.body2,
            bodyB=self.body3,
            localAnchorA=(0, -(fixture_length - 0.5)),
            localAnchorB=(0, fixture_length - 0.5),
            enableMotor=True,
            maxMotorTorque=400,
            enableLimit=False,
        )

        self.set_joint_angles(self.body1, self.body2, self.body3, x0[0], x0[1], x0[2] )
        self.set_joint_angles(self.target1, self.target2, self.target3, target[0], target[1], target[2] )
        self.target1.active = False
        self.target2.active = False

        self.joint1.motorSpeed = x0[3]
        self.joint2.motorSpeed = x0[4]
        self.joint3.motorSpeed = x0[5]

    def set_joint_angles(self, body1, body2, body3, angle1, angle2, angle3):
        """ Converts the given absolute angle of the arms to joint angles"""
        pos = self.base.GetWorldPoint((0, 0))
        body1.angle = angle1 + np.pi
        new_pos = body1.GetWorldPoint((0, 5))
        body1.position += pos - new_pos

        body2.angle = angle2 + body1.angle
        pos = body1.GetWorldPoint((0, -4.5))
        new_pos = body2.GetWorldPoint((0, 4.5))
        body2.position += pos - new_pos

        body3.angle = angle3 + body2.angle
        pos = body2.GetWorldPoint((0, -3.0))
        new_pos = body3.GetWorldPoint((0, 3.0))
        body3.position += pos - new_pos

    def Step(self, settings, action):
        """Moves forward in time one step."""
        self.joint1.motorSpeed = action[0]
        self.joint2.motorSpeed = action[1]
        self.joint3.motorSpeed = action[2]

        super(Arm3World, self).Step(settings)

    def reset_world(self):
        """Returns the world to its intial state"""
        self.world.ClearForces()
        self.joint1.motorSpeed = 0
        self.joint2.motorSpeed = 0
        self.joint3.motorSpeed = 0
        self.body1.linearVelocity = (0, 0)
        self.body1.angularVelocity = 0
        self.body2.linearVelocity = (0, 0)
        self.body2.angularVelocity = 0
        self.body3.linearVelocity = (0, 0)
        self.body3.angularVelocity = 0
        self.set_joint_angles(self.body1, self.body2, self.body3, self.x0[0], self.x0[1], self.x0[2] )


    def get_state(self):
        """Retrieves the state of the point mass"""
        state = {JOINT_ANGLES: np.array([self.joint1.angle,
                                         self.joint2.angle,
                                         self.joint3.angle]),
                 JOINT_VELOCITIES: np.array([self.joint1.speed,
                                             self.joint2.speed,
                                             self.joint3.speed])}

        return state

