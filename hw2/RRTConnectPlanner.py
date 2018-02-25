import numpy, operator
from RRTPlanner import RRTTree
import random

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        #plan.append(start_config)
        
        #RRT
        dist = float('inf')

        while dist > epsilon:
            # Bias Sampling with P of sampling towards the Goal = 0.1
            p = random.random()
            if p < 0.1:
                f_q_r = goal_config
            else:
                f_q_r = self.planning_env.GenerateRandomConfiguration()

            # Get the Vertex closest to q_r
            f_sid, f_q_n = ftree.GetNearestVertex(f_q_r)

            # Using linear interpolation to get the furthest vertex without collision
            f_q_c = self.planning_env.Extend(f_q_n, f_q_r)

            # Add vertex to the tree
            f_eid = ftree.AddVertex(f_q_c)

            # Add an edge on the tree
            ftree.AddEdge(f_sid, f_eid)

            if self.visualize:
                self.planning_env.PlotEdge(f_q_n, f_q_c)
            
            
            #for reverse tree
            r_sid, r_q_n = rtree.GetNearestVertex(f_q_c)

            # Using linear interpolation to get the furthest vertex without collision
            r_q_c = self.planning_env.Extend(r_q_n, f_q_c)  

            if(numpy.array_equal(r_q_n,r_q_c)):
                print("collision")

                
            # Add vertex to the r tree
            r_eid = rtree.AddVertex(r_q_c)

            # Add an edge on the r tree
            rtree.AddEdge(r_sid, r_eid)
            

            # Check how close we are to the goal
            dist = self.planning_env.ComputeDistance(f_q_c, r_q_c)
            print(dist)
            # print q_n
            print f_q_c, f_q_n, r_q_c, r_q_n 
            if self.visualize:
                self.planning_env.PlotEdge(r_q_n, r_q_c)

        print("trees constructed")
        vid = f_eid
        while (vid != ftree.GetRootId()):
            plan = [ftree.vertices[vid]] + plan
            vid = ftree.edges[vid]
            
        vid = r_eid
        while (vid != rtree.GetRootId()):
            vid = rtree.edges[vid]
            plan = plan + [rtree.vertices[vid]]           

        plan.append(goal_config)
        print("finished adding plan")
        print plan        
        return plan
