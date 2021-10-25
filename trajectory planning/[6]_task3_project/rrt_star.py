"""
机器人学规划仿真作业



ps:expandDis和math.sqrt((math.log(nnode) / nnode))前的数可调

"""

import matplotlib.pyplot as plt
import random
import math
import copy

show_animation = False


class Node(object):
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class RRT_STAR(object):
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacle_list, rand_area):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:random sampling Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expandDis = 500.0
        self.goalSampleRate = 0.05  # 选择终点的概率是0.05
        self.maxIter = 500
        self.obstacleList = obstacle_list
        self.nodeList = [self.start]
        self.connect_circle_dist=650,

    def random_node(self):
        """
        产生随机节点
        :return:
        """
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        :param node_list:
        :param rnd:
        :return:
        """
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        min_index = d_list.index(min(d_list))
        return min_index


    @staticmethod
    def collision_check(new_node, obstacle_list):
        a = 1
        for (ox, oy, size) in obstacle_list:
            dx = ox - new_node.x
            dy = oy - new_node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                a = 0  # collision

        return a  # safe
    
    def choose_parent(self, new_node, near_inds):

        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = list()
        for i in near_inds:
            near_node = self.nodeList[i]
            theta = math.atan2(new_node.y -  near_node.y, new_node.x - near_node.x)
            t_node = copy.deepcopy(near_node)
            t_node.x += self.expandDis * math.cos(theta)
            t_node.y += self.expandDis * math.sin(theta)
            if t_node and self.collision_check(t_node, self.obstacleList):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        theta = math.atan2(new_node.y - self.nodeList[min_ind].y, new_node.x - self.nodeList[min_ind].x)
        new_node = copy.deepcopy(self.nodeList[min_ind])
        new_node.x += self.expandDis * math.cos(theta)
        new_node.y += self.expandDis * math.sin(theta)
        new_node.parent = self.nodeList[min_ind]
        new_node.cost = min_cost

        return new_node
    
    def calc_new_cost(self, from_node, to_node):
        d=math.sqrt((from_node.x-to_node.x)**2+(from_node.y-to_node.y)**2)
        return from_node.cost + d       
    
    def find_near_nodes(self, new_node):
        nnode = len(self.nodeList) + 1
        r =800 * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expandDis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.nodeList]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds
    
    def rewire(self, new_node, near_inds):

        for i in near_inds:
            near_node = self.nodeList[i]
            theta = math.atan2(near_node.y - new_node.y, near_node.x - new_node.x)
            near_node_steer = copy.deepcopy(new_node)
            near_node_steer.x += self.expandDis * math.cos(theta)
            near_node_steer.y += self.expandDis * math.sin(theta)
            near_node_steer.parent = new_node
            if not near_node_steer:
                continue
            near_node_steer.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.collision_check(near_node_steer, self.obstacleList)
            improved_cost = near_node.cost > near_node_steer.cost

            if no_collision and improved_cost:
                near_node.x = near_node_steer.x
                near_node.y = near_node_steer.y
                near_node.cost = near_node_steer.cost
                near_node.parent = near_node_steer.parent


    def planning(self):
        
        while True:
            # Random Sampling
            if random.random() > self.goalSampleRate:
                rnd = self.random_node()
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            min_index = self.get_nearest_list_index(self.nodeList, rnd)
            # print(min_index)

            # expand tree
            nearest_node = self.nodeList[min_index]

            # 返回弧度制
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expandDis * math.cos(theta)
            new_node.y += self.expandDis * math.sin(theta)
            new_node.parent = nearest_node

            if not self.collision_check(new_node, self.obstacleList):
                continue
            
            near_inds= self.find_near_nodes(new_node)
            node_with_updated_parent = self.choose_parent(new_node, near_inds)
            if node_with_updated_parent:
                self.rewire(node_with_updated_parent, near_inds)
                self.nodeList.append(node_with_updated_parent)
                dx = node_with_updated_parent.x - self.end.x
                dy = node_with_updated_parent.y - self.end.y
            else:
                self.nodeList.append(new_node)
                dx = new_node.x - self.end.x
                dy = new_node.y - self.end.y

            # check goal
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break
            '''
            if True:
                self.draw_graph(rnd)
            '''
        path = [[self.end.x, self.end.y]]
        last_node=self.nodeList[-1]
        while last_node.parent:
            path.append([last_node.x, last_node.y])
            last_node = last_node.parent
        path.append([self.start.x, self.start.y])
        nodeList2=self.nodeList

        return path,nodeList2

if __name__ == '__main__':
    main()