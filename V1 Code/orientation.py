import math
import misc_functions
from constants import ZERO_Y, ZERO_Z

leg_x_seperation_mm = 221.0
foot_y_seperation_mm = ZERO_Z * 2
torso_zero_height_mm = ZERO_Y-10

left_front_foot_coords_WORLD = (leg_x_seperation_mm / 2, foot_y_seperation_mm / 2, 0)
right_front_foot_coords_WORLD = (leg_x_seperation_mm / 2, -foot_y_seperation_mm / 2, 0)
left_back_foot_coords_WORLD = (-leg_x_seperation_mm / 2, foot_y_seperation_mm / 2, 0)
right_back_foot_coords_WORLD = (-leg_x_seperation_mm / 2, -foot_y_seperation_mm / 2, 0)

torso_origin_coords_WORLD = (0, 0, torso_zero_height_mm)

torso_width_y_mm = 85.0
torso_height_z_mm = 48.0
torso_length_x_mm = leg_x_seperation_mm

foot_points = [
	left_front_foot_coords_WORLD,
	right_front_foot_coords_WORLD,
	left_back_foot_coords_WORLD,
	right_back_foot_coords_WORLD
]
torso_origin = torso_origin_coords_WORLD


		

def set_translation_orientation(translation, orientation):
	tx, ty, tz = translation
	yaw_deg, pitch_deg, roll_deg = orientation
	yaw = math.radians(yaw_deg)
	pitch = math.radians(pitch_deg)
	roll = math.radians(roll_deg)
	torso_origin_trans = misc_functions.vec_add(torso_origin, (tx, ty, tz))

	# Hip locations (relative to torso origin)
	l = torso_length_x_mm / 2
	w = torso_width_y_mm / 2
	h = 0
	hips_local = [
		( l,  w, h),  # left_front
		( l, -w, h),  # right_front
		(-l, -w, h),  # right_rear
		(-l,  w, h),  # left_rear
	]
	hip_labels = ['left front', 'right front', 'right rear', 'left rear']
	# Apply translation and rotation to hips
	hips_world = misc_functions.apply_rotation([misc_functions.vec_add(h, torso_origin_trans) for h in hips_local], torso_origin_trans, yaw, pitch, roll)

	# Always pair hips to their original feet (fixed mapping, with rear swapped)
	foot_points_arr = foot_points
	hip_to_foot_idx = [0, 1, 3, 2]

	# Hip axes (local)
	hip_axes_local = [
		[(-1, 0, 0), (0, 0, -1), (0, 1, 0)],  # left_front
		[(-1, 0, 0), (0, 0, -1), (0, -1, 0)], # right_front
		[(-1, 0, 0), (0, 0, -1), (0, -1, 0)], # right_rear
		[(-1, 0, 0), (0, 0, -1), (0, 1, 0)],  # left_rear
	]
	z_offsets = [torso_width_y_mm/2] * 4

	# Compute hip-to-foot vectors in hip local axes
	R = misc_functions.rotation_matrix(yaw, pitch, roll)
	vectors = []
	for i, idx in enumerate(hip_to_foot_idx):
		hip = hips_world[i]
		foot = foot_points_arr[idx]
		vec_world = misc_functions.vec_sub(foot, hip)
		axes = hip_axes_local[i]
		R_hip = tuple(misc_functions.mat_vec_mul(R, axis) for axis in axes)
		vec_local = [misc_functions.vec_dot(R_hip[j], vec_world) for j in range(3)]
		vec_local[2] += z_offsets[i]
		vec_local[0] = -vec_local[0]
		vectors.append(tuple(vec_local))
	# Return in the order: front_left, front_right, rear_right, rear_left
	return (vectors[0], vectors[1], vectors[2], vectors[3])
	
