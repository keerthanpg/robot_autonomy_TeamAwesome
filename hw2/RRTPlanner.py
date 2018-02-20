import numpy
import random
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        dist = float('inf')

        while dist > epsilon:
            # Bias Sampling with P of sampling towards the Goal = 0.1
            p = random.random()
            if p < 0.1:
                q_r = goal_config
            else:
                q_r = self.planning_env.GenerateRandomConfiguration()

            # Get the Vertex closest to q_r
            sid, q_n = tree.GetNearestVertex(q_r)

            # Using linear interpolation to get the furthest vertex without collision
            q_c = self.planning_env.Extend(q_n, q_r)

            # Add vertex to the tree
            eid = tree.AddVertex(q_c)

            # Add an edge on the tree
            tree.AddEdge(sid, eid)

            # Check how close we are to the goal
            dist = self.planning_env.ComputeDistance(q_c, goal_config)

            if self.visualize:
                # print q_n
                # print q_c
                self.planning_env.PlotEdge(q_n, q_c)


        # plan.append(start_config)
        # plan.append(goal_config)
        vid = eid
        while (vid != tree.GetRootId()):
            vid = tree.edges[vid]
            plan = [tree.vertices[vid]] + plan

        plan.append(goal_config)
        return plan
