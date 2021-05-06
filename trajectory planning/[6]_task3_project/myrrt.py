from math import sqrt
import random
import math
import copy
from rrt_star import RRT_STAR

from numpy.lib.function_base import append

class Node :
    """
    Node for rrt planning
    """
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

class RRT_STAR :
    def __init__(self, start, end, obstacle_list):
        self.start = Node(start[0], start[1])
        self.end = Node(end[0], end[1])
        self.obstacle_list=obstacle_list  #元素为[ox,oy,size]
        self.steplength=600
        self.nodeList=[self.start]
    
    def return_random_node(self):
        x=random.uniform(-3500,3500)
        y=random.uniform(-3500,3500)
        return Node(x,y)

    def get_nearest_node(self,new_node):
        distance2=list()
        for node in self.nodeList:
            distance2.append((new_node.x-node.x)**2 + (new_node.y-node.y)**2)
        return self.nodeList[distance2.index(min(distance2))]

    def steer(self,from_node,to_node):
        angle = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = copy.deepcopy(from_node)
        new_node.x += self.steplength * math.cos(angle)
        new_node.y += self.steplength * math.sin(angle)
        new_node.parent = from_node
        new_node.cost = from_node.cost + self.steplength
        return new_node


    def check_collision(self,node_new):
        
        for obstacle in self.obstacle_list:
            d2 = (node_new.x-obstacle[0])**2 +(node_new.y-obstacle[1])**2
            if d2 < obstacle[2]**2:
                return True
        return False

    def check_collision2(self,node_old,node_new):
        flag=self.check_collision(node_new)
        node=Node(node_old.x/2 + node_new.x/2,node_old.y/2 + node_new.y/2)
        flag=self.check_collision(node)
        node2=Node(node_old.x*0.75 + node_new.x*0.25,node_old.y*0.75 + node_new.y*0.25)
        flag=self.check_collision(node2)
        node3=Node(node_old.x*0.25 + node_new.x*0.75,node_old.y*0.25 + node_new.y*0.75)
        flag=self.check_collision(node3)
        return flag

    def choose_parent(self,new_node):

        near=self.choose_near(new_node)
        self.near=near
        if not near:
            return None

        costlist=[node.cost + math.sqrt((node.x-new_node.x)**2+(node.y-new_node.y)**2) for node in near]
        min_index = costlist.index(min(costlist))
        near_chosen = self.near[min_index]
        if self.check_collision2(near_chosen,new_node):
            return None
        new_node_with_updated_parent = copy.deepcopy(new_node)
        new_node_with_updated_parent.parent = near_chosen
        new_node_with_updated_parent.cost = near_chosen.cost + costlist[min_index]
        return new_node_with_updated_parent

    def choose_near(self,new_node):
        near=list()
        for node in self.nodeList:
            if math.sqrt((node.x-new_node.x)**2+(node.y-new_node.y)**2) <= self.steplength* 1.5 :
                near.append(node)
        return near

    def rewire(self,new_node):
        for node in self.near:
            if (math.sqrt((node.x-new_node.x)**2+(node.y-new_node.y)**2) + new_node.cost < node.cost) & ( ~ self.check_collision2(node,new_node)):
                node.parent=new_node
                node.cost=new_node.cost+ math.sqrt((node.x-new_node.x)**2+(node.y-new_node.y)**2)


            
        


    def planning(self):


        while True:
            if random.random() > 0.05 :
                new_random_node=self.return_random_node() 
            else:
                new_random_node=self.end     #改进：直接以终点为目标

            nearest_node=self.get_nearest_node(new_random_node)

            new_node=self.steer(nearest_node,new_random_node)

            if self.check_collision2(nearest_node,new_node):
                continue


            new_node_with_updated_parent=self.choose_parent(new_node)
            if new_node_with_updated_parent:
                new_node = new_node_with_updated_parent
                self.rewire(new_node)

            self.nodeList.append(new_node)

            d=(new_node.x-self.end.x)**2 +(new_node.y-self.end.y)**2
            if d<=self.steplength**2:
                print("Find Goal!")
                break

        path = [[self.end.x, self.end.y]]
        last_node=self.nodeList[-1]
        while last_node.parent:
            path.append([last_node.x, last_node.y])
            last_node = last_node.parent
        path.append([self.start.x, self.start.y])
        nodeList2=self.nodeList
        
        return path,nodeList2









if __name__ == '__main__':
   print(1)
