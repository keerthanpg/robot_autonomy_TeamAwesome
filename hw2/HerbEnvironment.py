import numpy
import random
import time
import copy

class HerbEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def CheckCollision(self, config):
        # transform = numpy.identity(4)
        # transform[0, 3] = config[0]
        # transform[1, 3] = config[1]
        # self.robot.SetTransform(transform)
        # prev_conf = self.robot.GetActiveDOFValues()
        with self.robot:
            self.robot.SetActiveDOFValues(config)
            collide = self.robot.GetEnv().CheckCollision(self.robot)
        # self.robot.SetActiveDOFValues(prev_conf)
        # transform[0, 3] = -config[0]
        # transform[1, 3] = -config[1]
        # self.robot.SetTransform(transform)
        return collide

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        # print self.robot.GetActiveDOFLimits()
        # print self.robot.GetActiveDOFLimits()
        #
        # TODO: Generate and return a random configuration
        #
        collide = True
        while collide:
            config[0] = random.random() * abs(lower_limits[0] - upper_limits[0]) + lower_limits[0]
            config[1] = random.random() * abs(lower_limits[1] - upper_limits[1]) + lower_limits[1]
            config[2] = random.random() * abs(lower_limits[2] - upper_limits[2]) + lower_limits[2]
            config[3] = random.random() * abs(lower_limits[3] - upper_limits[3]) + lower_limits[3]
            config[4] = random.random() * abs(lower_limits[4] - upper_limits[4]) + lower_limits[4]
            config[5] = random.random() * abs(lower_limits[5] - upper_limits[5]) + lower_limits[5]
            config[6] = random.random() * abs(lower_limits[6] - upper_limits[6]) + lower_limits[6]
            collide = self.CheckCollision(config)

        return numpy.array(config)



    def ComputeDistance(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        distance = numpy.sqrt(numpy.sum(numpy.square(start_config - end_config)))

        return distance


    def Extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        # y_coord = numpy.interp(x_coord, [start_config[0], end_config[0]], [start_config[1], end_config[1]])
        # number of interpolate points
        dof = len(self.robot.GetActiveDOFIndices())
        num = 500
        positions = numpy.zeros((dof,num))
        positions[0,:] = numpy.linspace(start_config[0], end_config[0], num)
        for i in range(1,dof):
            positions[i,:] = numpy.interp(positions[0,:] , [start_config[0], end_config[0]], [start_config[i], end_config[i]])

        config = numpy.copy(start_config)
        for i in range(num):
            if not self.CheckCollision(positions[:,i]):
                config = numpy.copy(positions[:,i])
            else:
                break
        return config

    def ShortenPath(self, path, timeout=5.0):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        return path
