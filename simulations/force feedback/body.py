from units import *
from constants import *
from linkage import *
import copy
import matplotlib.pyplot as plt

class Body:
    def __init__(self, mass: Mass, wheelbase_mm: int, leg: Linkage):
        self.mass = mass
        self.wheelbase_mm = wheelbase_mm
        self.cog = Coordinate.origin()

        self.front_leg = leg
        self.rear_leg = copy.deepcopy(leg)
        self.legs = [self.front_leg, self.rear_leg]
        self.front_leg.id = "front leg"
        self.rear_leg.id = "rear leg"
        self.weight = ForceVector.gravity(mass)

        self.front_leg.translate(Coordinate(-self.wheelbase_mm/2, 0))
        self.rear_leg.translate(Coordinate(self.wheelbase_mm/2, 0))

        print(f"[{self.front_leg.id}] foot position: {self.front_leg.foot.position.x}, {self.front_leg.foot.position.y}")
        print(f"[{self.rear_leg.id}] foot position: {self.rear_leg.foot.position.x}, {self.rear_leg.foot.position.y}")

        


    def solve_robot(self):
        last_reactions = None
        reactions = ForceVector.find_vertical_reactions([[self.weight, self.cog]], [self.front_leg.foot.position, self.rear_leg.foot.position])

        while self.reactions_not_converged(reactions, last_reactions):
            print("iterating")

            last_reactions = reactions.copy()
            self.front_leg.foot.reaction = reactions[0][0]
            self.rear_leg.foot.reaction = reactions[1][0]

            for leg in self.legs:
                link_directions: list[Angle] = []
                for link in leg.foot.links:
                    link_directions.append(link.direction)
                link_force_vectors = ForceVector.find_equilibirum_vectors([leg.foot.reaction], link_directions)
                for link in leg.foot.links:
                    if link_force_vectors[0].dir.rad == link.direction.rad:
                        link.load = link_force_vectors[0]
                    elif link_force_vectors[1].dir.rad == link.direction.rad:
                        link.load = link_force_vectors[1]

                    
                    link.update_length()
                    print(f"Leg: [{leg.id}], link: [{link.id}], load direction: {link.load.dir.deg} magnitude: {link.load.mag.tension_N}, lew length: {link.length}")

            torso_angle = Angle.from_degrees(0)
            target_foot_positions: list[Coordinate] = []

            for leg in self.legs:
                target_foot_positions.append(leg.foot.position)
        
            for leg in self.legs:
                if leg == self.front_leg:
                    mulitplier = 1
                else:
                    mulitplier = -1
                leg.joints[0].position.x = self.cog.x - mulitplier * (((wheelbase_mm + servo_seperation_mm)/2) * np.cos(torso_angle.rad))
                leg.joints[0].position.y = self.cog.y - mulitplier * (((wheelbase_mm + servo_seperation_mm)/2) * np.sin(torso_angle.rad))
                leg.joints[-1].position.x = self.cog.x - mulitplier * (((wheelbase_mm - servo_seperation_mm)/2) * np.cos(torso_angle.rad))
                leg.joints[-1].position.y = self.cog.y - mulitplier * (((wheelbase_mm - servo_seperation_mm)/2) * np.sin(torso_angle.rad))

                if leg == self.front_leg:
                    leg.links[0].direction.from_degrees(np.degrees(np.pi) + torso_angle.deg - leg.links[0].direction.deg)
                    leg.links[-1].direction.from_degrees(leg.links[-1].direction.deg - torso_angle.deg)
                
                else:
                    leg.links[0].direction = Angle.from_degrees(np.degrees(np.pi) + torso_angle.deg - leg.links[0].direction.deg)
                    leg.links[-1].direction = Angle.inverse(Angle.from_degrees(leg.links[-1].direction.deg - torso_angle.deg))

                leg.joints[1].position = Coordinate.find_end_point(leg.joints[0].position, leg.links[0].length, leg.links[0].direction)
                leg.joints[-2].position = Coordinate.find_end_point(leg.joints[-1].position, leg.links[-1].length, leg.links[-1].direction)

                calf1_circle = Circle(leg.joints[1].position, leg.links[1].length)
                calf2_circle = Circle(leg.joints[-2].position, leg.links[-2].length)

                intersections = Circle.intersection_between_circles(calf1_circle, calf2_circle)
                leg.foot.position = Coordinate.get_lower_point(intersections)
            
            left_foot_error = Coordinate.distance_between_points(target_foot_positions[0], self.legs[0].foot.position)
            self.translate(left_foot_error)

            feet_seperation = Coordinate.distance_between_points(self.legs[1].foot.position, self.legs[0].foot.position)
            torso_angle_error = Angle.from_radians(np.atan2(feet_seperation.y,feet_seperation.x)*2)
            self.rotate(self.legs[0].foot.position, torso_angle_error)

            reactions = ForceVector.find_vertical_reactions([[self.weight, self.cog]], [self.front_leg.foot.position, self.rear_leg.foot.position])
        
        print("Converged")
        print(f"[{self.front_leg.id}] foot position: {self.front_leg.foot.position.x}, {self.front_leg.foot.position.y}")
        print(f"[{self.rear_leg.id}] foot position: {self.rear_leg.foot.position.x}, {self.rear_leg.foot.position.y}")





            


    @staticmethod
    def reactions_not_converged(current_reactions: list[tuple[ForceVector, Coordinate]], last_reactions: list[tuple[ForceVector, Coordinate]] | None) -> bool:
        if last_reactions is None:
            return True
        threshold = 0.1
        for i in range (len(current_reactions)):
            if np.abs(current_reactions[i][0].dir.rad) - np.abs(last_reactions[i][0].dir.rad) > threshold:
                print(f"Reactions directions don't match")
                return True
            if np.abs(current_reactions[i][0].mag.tension_N) - np.abs(last_reactions[i][0].mag.tension_N) > threshold:
                print("Reaction Magntides don't match")
                return True
            if np.abs(current_reactions[i][1].x) - np.abs(last_reactions[i][1].x) > threshold:
                print("Reaction x coords don't match")
                return True
            if np.abs(current_reactions[i][1].y) - np.abs(last_reactions[i][1].y) > threshold:
                print("Reaction y coords don't match")
                return True
            
        return False

        
    def translate(self, delta: Coordinate):
        for leg in self.legs:
            leg.translate(delta)
        self.cog = Coordinate.translate(self.cog, delta)

    def rotate(self, centre: Coordinate, angle: Angle):
        for leg in self.legs:
            leg.rotate(centre, angle)
        self.cog = Coordinate.rotate(self.cog, centre, angle)


            #now both linkages and the torso line have been translated, need to rotate them both until the second foot error = 0
            #can do this iteratively but be better to do some math really 


        #torso centre has now been set, now need to t
    


        


    


        




    def draw_body(self):
        x_coords = []
        y_coords = []
        colors = []

        for leg in self.legs:
            for joint in leg.joints:
                x_coords.append(joint.position.x)
                y_coords.append(joint.position.y)
            
                if joint.id == "foot":
                    colors.append('red')
                elif joint.id == "front knee" or joint.id == "rear knee":
                    colors.append('green') 
                else:
                    colors.append('blue')

        colors.append('black')
        x_coords.append(self.cog.x)
        y_coords.append(self.cog.y)

                

        plt.scatter(x_coords, y_coords, c=colors)
        plt.axis('equal')
        plt.show()
        