from units import *
from constants import *
from linkage import *
import copy
import matplotlib.pyplot as plt
from scipy.optimize import fsolve, least_squares

class Body:
    def __init__(self, mass: Mass, wheelbase_mm: int, leg: Linkage, convergence_threshold: float = 0.1):
        self.mass = mass
        self.wheelbase_mm = wheelbase_mm
        self.cog = Coordinate.origin()
        self.torso_angle = Angle.from_degrees(0)
        self.convergence_threshold = convergence_threshold
        self.history: list[dict] = []

        self.front_leg = leg
        self.rear_leg = copy.deepcopy(leg)
        self.legs = [self.front_leg, self.rear_leg]
        self.front_leg.id = "front leg"
        self.rear_leg.id = "rear leg"
        self.weight = ForceVector.gravity(mass)

        self.front_leg.translate(Coordinate(-self.wheelbase_mm/2, 0))
        self.rear_leg.translate(Coordinate(self.wheelbase_mm/2, 0))
        self.translate(Coordinate(0, -self.rear_leg.foot.position.y))

        print(f"COG: ({self.cog.x}, {self.cog.y})")
        print(f"[{self.front_leg.id}] foot position: {self.front_leg.foot.position.x}, {self.front_leg.foot.position.y}")
        print(f"[{self.rear_leg.id}] foot position: {self.rear_leg.foot.position.x}, {self.rear_leg.foot.position.y}")

        

        initial_reactions = ForceVector.find_vertical_reactions(
            [[self.weight, self.cog]],
            [self.front_leg.foot.position, self.rear_leg.foot.position],
        )
        self._snapshot(initial_reactions, label="initial")
    
    def get_feet_positions(self):
        foot_positions = []
        for leg in self.legs:
            foot_positions.append(leg.foot.position)

        return foot_positions
    
    def get_knee_positions(self, cog: Coordinate, input_torso_angle: Angle) -> list[Coordinate]:
        knee_positions = []

        half_torso_length = self.wheelbase_mm/2
        thigh_length = self.front_leg.links[0].length
        torso_angle = input_torso_angle.rad
        x = cog.x
        y = cog.y

        for leg in self.legs:
            thigh_angles = [leg.links[0].direction, leg.links[-1].direction]
            direction = 1 if leg == self.front_leg else -1
            for i in range(len(thigh_angles)):
                knee_positions.append(
                    Coordinate(x + thigh_length * np.cos(torso_angle + thigh_angles[i].rad) - half_torso_length * np.cos(torso_angle) * direction,
                               y + thigh_length * np.sin(torso_angle + thigh_angles[i].rad) - half_torso_length * np.sin(torso_angle) * direction))

        return knee_positions

    def get_thigh_forces(self, knee_positions: list[Coordinate], feet_positions: list[Coordinate]) -> list[ForceVector]:
        calf_lengths = []
        calf_directions = []
        thigh_forces = []
        knee_offset = 0
        for i in range(len(feet_positions)):
            calf_lengths.append(np.hypot(feet_positions[i].x - knee_positions[i + knee_offset].x, feet_positions[i].y - knee_positions[i + knee_offset].y))
            calf_lengths.append(np.hypot(feet_positions[i].x - knee_positions[i + 1 + knee_offset].x, feet_positions[i].y - knee_positions[i + 1 + knee_offset].y))

            calf_directions.append(Coordinate.get_angle_from_points(feet_positions[i], knee_positions[i + knee_offset]))
            calf_directions.append(Coordinate.get_angle_from_points(feet_positions[i], knee_positions[i + 1 + knee_offset]))

            knee_offset = 1
        
        for i in range(len(calf_lengths)):
            extension = calf_length_mm - calf_lengths[i]
            thigh_forces.append(ForceVector(calf_directions[i], Tension.from_newtons(extension * spring_rate_N_per_mm)))
        
        return thigh_forces

            


    def residuals(self, params):
        x, y, theta = params
        cog = Coordinate(x, y)
        torso_angle = Angle.from_radians(theta)
        relative_knee_positions = self.get_knee_positions(cog, torso_angle)
        thigh_forces = self.get_thigh_forces(relative_knee_positions, self.get_feet_positions())
        sum_forces = ForceVector.sum_forces(thigh_forces) + self.weight

        torso_torque = Torque.from_Nmm(sum(np.cross([relative_knee_positions[i].x, relative_knee_positions[i].y], [thigh_forces[i].x_component, thigh_forces[i].y_component]) for i in range(4)))

        return [sum_forces.x_component, sum_forces.y_component, torso_torque.torque_Nmm]
    
    
    def solve_fsolve(self):
        sum_foot_x = 0
        for leg in self.legs:
            sum_foot_x = sum_foot_x + leg.foot.position.x
        
        print(f"Initial CoG Guess: ({sum_foot_x/2}, {self.cog.y})")
        

        initial_guess = [
            0,
            self.cog.y*2,
            np.radians(45)
        ]


        result, info, ier, mesg = fsolve(
            lambda params: self.residuals(params),
            initial_guess,
            full_output=True
        )

        self.cog = Coordinate(result[0], result[1])
        self.torso_angle = Angle.from_radians(result[2])

        print(f"Solved!, CoG: ({self.cog.x}, {self.cog.y})")
        print(f"Torso angle: {self.torso_angle.deg}°")

        print(f"Function evaluations: {info['nfev']}")
        print(f"Jacobian evaluations: {info.get('njev', 'N/A')}")
        print(f"Converged: {ier == 1}")
        print(f"Message: {mesg}")


    def solve_least_squares(self):

        sum_foot_x = 0
        for leg in self.legs:
            sum_foot_x = sum_foot_x + leg.foot.position.x
        
        print(f"Initial CoG Guess: ({sum_foot_x/2}, {self.cog.y})")

        initial_guess = np.array([
            sum_foot_x/2,
            self.cog.y,
            self.torso_angle.rad
        ])

        result = least_squares(
            self.residuals,
            initial_guess,
            method='lm',
            ftol=1e-15,
            xtol=1e-15,
            gtol=1e-15,
            max_nfev=2000
        )

        # update state
        self.cog = Coordinate(result.x[0], result.x[1])
        self.torso_angle = Angle.from_radians(result.x[2])

        # diagnostics
        print(f"Solved!")
        print(f"CoG: ({self.cog.x:.6f}, {self.cog.y:.6f})")
        print(f"Torso angle: {self.torso_angle.deg:.3f}°")

        print("\n--- Optimisation info ---")
        print(f"Cost (0.5 * ||res||^2): {result.cost}")
        print(f"Residual norm: {np.linalg.norm(result.fun)}")
        print(f"Function evals: {result.nfev}")
        print(f"Jacobian evals: {result.njev}")
        print(f"Status: {result.status}")
        print(f"Message: {result.message}")


        


        


    def solve_robot(self):
        last_reactions = None
        reactions = ForceVector.find_vertical_reactions([[self.weight, self.cog]], [self.front_leg.foot.position, self.rear_leg.foot.position])

        while self.reactions_not_converged(reactions, last_reactions, self.convergence_threshold):
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

            target_foot_positions: list[Coordinate] = []

            for leg in self.legs:
                target_foot_positions.append(Coordinate(leg.foot.position.x, leg.foot.position.y))

            for leg in self.legs:
                calf1_circle = Circle(leg.joints[1].position, leg.links[1].length)
                calf2_circle = Circle(leg.joints[-2].position, leg.links[-2].length)

                intersections = Circle.intersection_between_circles(calf1_circle, calf2_circle)
                leg.foot.position = Coordinate.get_lower_point(intersections)
            
            left_foot_error = Coordinate.distance_between_points(target_foot_positions[0], self.legs[0].foot.position)
            self.translate(left_foot_error)

            feet_seperation = Coordinate.distance_between_points(self.legs[1].foot.position, self.legs[0].foot.position)
            torso_angle_error = Angle.from_radians(np.atan2(feet_seperation.y,feet_seperation.x))
            self.rotate(self.legs[0].foot.position, torso_angle_error)

            reactions = ForceVector.find_vertical_reactions([[self.weight, self.cog]], [self.front_leg.foot.position, self.rear_leg.foot.position])
            self._snapshot(reactions, label=f"iter {len(self.history)}")

        print("Converged")
        print(f"COG: ({self.cog.x}, {self.cog.y})")
        print(f"Torso Angle: {self.torso_angle.deg}°")
        print(f"[{self.front_leg.id}] foot position: {self.front_leg.foot.position.x}, {self.front_leg.foot.position.y}")
        print(f"[{self.rear_leg.id}] foot position: {self.rear_leg.foot.position.x}, {self.rear_leg.foot.position.y}")

    @staticmethod
    def reactions_not_converged(current_reactions: list[tuple[ForceVector, Coordinate]], last_reactions: list[tuple[ForceVector, Coordinate]] | None, threshold: float = 0.1) -> bool:
        if last_reactions is None:
            return True
        for i in range (len(current_reactions)):
            if np.abs(current_reactions[i][0].dir.rad) - np.abs(last_reactions[i][0].dir.rad) > threshold:
                print(f"Reactions directions don't match")
                return True
            if np.abs(current_reactions[i][0].mag.tension_N) - np.abs(last_reactions[i][0].mag.tension_N) > threshold:
                print("Reaction Magnitudes don't match")
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
        self.torso_angle = Angle.from_radians(self.torso_angle.rad + angle.rad)

    def _snapshot(self, reactions, label: str = ""):
        legs_data = []
        for leg in self.legs:
            joints_xy = [(j.position.x, j.position.y) for j in leg.joints]
            links_data = []
            for link in leg.links:
                links_data.append({
                    "id": link.id,
                    "length": link.length,
                    "natural_length": link.natural_length,
                    "load_N": link.load.mag.tension_N,
                    "direction_deg": link.direction.deg if link.direction is not None else 0.0,
                })
            legs_data.append({
                "id": leg.id,
                "joints": joints_xy,
                "links": links_data,
                "foot": (leg.foot.position.x, leg.foot.position.y),
            })
        self.history.append({
            "label": label,
            "cog": (self.cog.x, self.cog.y),
            "torso_angle_deg": self.torso_angle.deg,
            "legs": legs_data,
            "reactions_N": [r[0].y_component for r in reactions],
        })

  