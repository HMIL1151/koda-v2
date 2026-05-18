from link import *
from units import *
from math_funcs import *
import matplotlib.pyplot as plt

class Linkage():
    def __init__ (self, links: list[Link]):
        self.input_links = links
        self.joints = []
        self.links = []
        
        self.populate_linkage()
        self.solve_forward_kinematics()
        
    
    def find_starting_joint(self):
        for link in self.input_links:
            for joint in link.joints:
                if joint.type == JointType.DRIVEN_STATIC:
                    self.joints.append(joint)
                    break
            else:
                continue
            break

        if len(self.joints) == 0:
            raise ValueError("No Driven Joint in Linkage")

    def populate_linkage(self):
        self.find_starting_joint()
        while len(self.links) < len(self.input_links):
            for link in self.input_links:
                for joint in link.joints:
                    if joint == self.joints[-1]:
                        if link not in self.links:
                            self.links.append(link)
                        else:
                            raise ValueError(f"Error: Link [{link.id}] is already in link list: {[l.id for l in self.links]}, joint list: {[j.id for j in self.joints]}")

                        other_joint = Link.get_other_joint(link, joint)
                        if other_joint not in self.joints:
                            self.joints.append(other_joint)

                        break
        self.add_links_to_joints()
        self.validate_linkage()

    def add_links_to_joints(self):
        for joint in self.joints:
            for link in self.links:
                for link_joint in link.joints:
                    if joint == link_joint:
                        joint.links.append(link)
    
    def print_linkage(self):
        for i in range (len(self.joints)):
            print(f"joint#{i}: {self.joints[i].id}")
            if i < len(self.links):
                print(f"link#{i}: {self.links[i].id}")

    def validate_linkage(self):
        num_joints = len(self.joints)
        num_links = len(self.links)
        if num_joints != num_links + 1:
            self.print_linkage()
            raise ValueError(f"link and joint count incorrect, Joints = {num_joints}, Links = {num_links}")
        print("Linkage Validated")

    def set_link_loads(self, link_tensions: list[tuple[Link, Tension]]):
        for link_tension in link_tensions:
            link_tension[0].set_load(link_tension[1])
        self.solve_forward_kinematics()

       
    def solve_forward_kinematics(self):

        for link in self.links:
            if link.driven == True and link.position_set == False:
                raise ValueError(f"driven link {link.id} angle is not set")
            
        if self.joints[-1].type != JointType.DRIVEN_STATIC or self.joints[0].type != JointType.DRIVEN_STATIC:
            raise ValueError("start and end joints are not static")
        
        if len(self.links) > 4:
            raise ValueError(f"problem is too indeterminate for this solver")
        
        circle1 = Circle(self.joints[1].position, self.links[1].length)
        circle2 = Circle(self.joints[-2].position, self.links[-2].length)

        intersections = intersection_between_circles(circle1, circle2)

        if len(intersections) < 1:
            raise ValueError("No intersections")
        
        joint_position = intersections[0]
        for intersection in intersections:
            if intersection.y < joint_position.y:
                joint_position = intersection

        self.joints[2].set_position(joint_position)
        return
    
    def draw_linkage(self):
        x_coords = []
        y_coords = []
        for joint in self.joints:
            x_coords.append(joint.position.x)
            y_coords.append(joint.position.y)
        
        plt.plot(x_coords, y_coords)
        plt.axis('equal')
        plt.show()
