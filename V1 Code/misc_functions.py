import math

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def interpolate_between_servo_positions(start_pos, end_pos, steps):
    start_angle_hip = start_pos[0]
    start_angle_front = start_pos[1]
    start_angle_rear = start_pos[2]

    end_angle_hip = end_pos[0]
    end_angle_front = end_pos[1]
    end_angle_rear = end_pos[2]

    return [
        (
            int(start_angle_hip + (end_angle_hip - start_angle_hip) * i / steps),
            int(start_angle_front + (end_angle_front - start_angle_front) * i / steps),
            int(start_angle_rear + (end_angle_rear - start_angle_rear) * i / steps)
        )
        for i in range(steps + 1)
    ]

def vec_add(a, b):
	return tuple(x + y for x, y in zip(a, b))

def vec_sub(a, b):
	return tuple(x - y for x, y in zip(a, b))

def vec_mul(a, scalar):
	return tuple(x * scalar for x in a)

def vec_div(a, scalar):
	return tuple(x / scalar for x in a)

def vec_dot(a, b):
	return sum(x * y for x, y in zip(a, b))

def vec_norm(a):
	return math.sqrt(sum(x * x for x in a))

def vec_distance(a, b):
	return vec_norm(vec_sub(a, b))

def mat_vec_mul(mat, vec):
	return tuple(sum(mat[i][j] * vec[j] for j in range(3)) for i in range(3))

def mat_mul(a, b):
	# Matrix multiplication (3x3)
	return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))

def transpose(mat):
	return tuple(tuple(mat[j][i] for j in range(3)) for i in range(3))

def make_eye3():
	return ((1,0,0),(0,1,0),(0,0,1))

def apply_rotation(points, center, yaw, pitch, roll):
	R = rotation_matrix(yaw, pitch, roll)
	rotated = []
	for pt in points:
		shifted = vec_sub(pt, center)
		rotated_pt = mat_vec_mul(R, shifted)
		rotated.append(vec_add(rotated_pt, center))
	return rotated

def rotation_matrix(yaw, pitch, roll):
	cy, sy = math.cos(yaw), math.sin(yaw)
	cp, sp = math.cos(pitch), math.sin(pitch)
	cr, sr = math.cos(roll), math.sin(roll)
	Rz = (
		(cy, -sy, 0),
		(sy,  cy, 0),
		( 0,   0, 1)
	)
	Ry = (
		(cp, 0, sp),
		( 0, 1,  0),
		(-sp,0, cp)
	)
	Rx = (
		(1,  0,   0),
		(0, cr, -sr),
		(0, sr,  cr)
	)
	return mat_mul(mat_mul(Rz, Ry), Rx)

def get_torso_corners(center, length, width, height):
	l = length / 2
	w = width / 2
	h = height / 2
	corners = [
		( l,  w,  h),
		( l, -w,  h),
		(-l, -w,  h),
		(-l,  w,  h),
		( l,  w, -h),
		( l, -w, -h),
		(-l, -w, -h),
		(-l,  w, -h),
	]
	return [vec_add(c, center) for c in corners]
