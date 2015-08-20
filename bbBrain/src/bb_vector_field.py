# vector field used to determine foot swing trajectory


from math import sqrt, atan, sin, cos


# size and resolution of vector field
MIN_Y = -0.3
MAX_Y = 1.3

MIN_Z = -0.3
MAX_Z = 1.3

Y_STEPS = 50
Z_STEPS = 50

current_effort = 0.1

field_forward = list()  # pushes foot forward to Anterior Extreme Position (AEP)
field_accel_limit = list()  # limits horizontal acceleration of foot

field_pep = list()  # push foot up and slightly back to make sure that field_forward is compensated and foot goes straight up when starting swing
field_boost = list()  # accelerate foot up as it departs from PEP
field_z_top = list()  # push foot down when it is lifted too high
field_center = list()  # push foot away from swing area center, forcing "ballistic" trajectory
field_corner = list()  # push foot down and decrease its horizontal speed when it is high above AEP
field_aep = list()  # lowers foot when close to AEP
field_overshoot = list()  # retracts foot when it gets beyond AEP in horizontal direction
field_soft_touchdown = list()  # slows downwards foot movement when close to expected touchdown
field_aggregate = list()  # combination of all the sub-fields


def empty_field():
    return [[(0, 0) for cy in range(Y_STEPS)] for rz in range(Z_STEPS)]


def fill_fields():
    global field_forward, field_accel_limit, field_pep, field_boost, field_z_top, field_center, field_corner, field_aep, field_overshoot, field_soft_touchdown, field_aggregate

    global current_effort
    fwd_effort = 4 + 12 * current_effort
    raise_effort = 3 + 7 * current_effort
    touchdown_rate = 0.3 + 0.2 * current_effort

    field_forward = constant_field((fwd_effort, 0))
    field_pep = elliptic_field(0, 0.7, 0, 1, (-fwd_effort * 0.9, raise_effort))
    field_boost = elliptic_field(0.2, 0.5, 0.5, 0.8, (-2, 4))
    field_z_top = rectangular_field(-0.3, 1, 1.3, 0.9, (0, 1), (0, -raise_effort))
    field_center = radial_field(0.5, 0.5, 0, 1, 2)
    field_corner = elliptic_field(1, 0.5, 1, 0.5, (-0.4, -1))
    field_aep = rectangular_field(1, 0.5, -0.3, 1.3, (1, 0), (-fwd_effort, -raise_effort * touchdown_rate))
    field_overshoot = rectangular_field(1.3, 1, -0.3, 1.3, (0, 0), (-fwd_effort, -raise_effort * touchdown_rate))
    field_soft_touchdown = rectangular_field(0.84, 1.3, -0.3, 0.3, (0, 1), (-1, 1))

    field_accel_limit = rectangular_field(0.4, -.3, -0.3, 1.3, (1, 0), (1, 0))

    field_aggregate = empty_field()

    field_aggregate = add_fields(field_aggregate, field_forward)
    field_aggregate = add_fields(field_aggregate, field_pep)
    field_aggregate = add_fields(field_aggregate, field_boost)
    field_aggregate = add_fields(field_aggregate, field_z_top)
    field_aggregate = add_fields(field_aggregate, field_center)
    field_aggregate = add_fields(field_aggregate, field_corner)
    field_aggregate = add_fields(field_aggregate, field_aep)
    field_aggregate = add_fields(field_aggregate, field_overshoot)
    field_aggregate = add_fields(field_aggregate, field_soft_touchdown)

    field_aggregate = scale_field(field_aggregate, field_accel_limit, ignore_zero=True)


def add_fields(f1, f2):
    resulting_field = empty_field()

    for r in range(Z_STEPS):
        for c in range(Y_STEPS):
            vctr = f1[r][c]
            vctr = add_vectors(vctr, f2[r][c])

            resulting_field[r][c] = vctr

    return resulting_field


def scale_field(field, scaler_field, ignore_zero):
    resulting_field = empty_field()

    for r in range(Z_STEPS):
        for c in range(Y_STEPS):
            vctr = field[r][c]
            factor_vector = scaler_field[r][c]

            vctr = scale_vector_by_vector(vctr, factor_vector, ignore_zero)

            resulting_field[r][c] = vctr

    return resulting_field


def get_row_col(y_n, z_n):
    y_step = (MAX_Y - MIN_Y) / Y_STEPS
    z_step = (MAX_Z - MIN_Z) / Z_STEPS

    dy = y_n - MIN_Y
    y_pos = int(dy / y_step)

    dz = z_n - MIN_Z
    z_pos = int(dz / z_step)

    return z_pos, y_pos


def get_vector(y_n, z_n):
    r, c = get_row_col(y_n, z_n)
    return field_aggregate[r][c]


def add_vectors(v1, v2):
    return v1[0] + v2[0], v1[1] + v2[1]


def scale_vector(v, factor):
    return v[0] * factor, v[1] * factor


def scale_vector_by_vector(v, factor_vector, ignore_zero):
    v0 = v[0]
    v1 = v[1]
    if factor_vector[0] != 0 or not ignore_zero:
        v0 *= factor_vector[0]
    else:
        pass

    if factor_vector[1] != 0 or not ignore_zero:
        v1 *= factor_vector[1]
    else:
        pass

    return v0, v1


def ellipse_border_point(h_radius, v_radius, direction_vector):
    a = direction_vector[0] / h_radius
    b = direction_vector[1] / v_radius
    k = sqrt(a ** 2 + b ** 2)

    return (direction_vector[0] / k), (direction_vector[1] / k)


def rectangular_field(y_from, y_to, z_from, z_to, attenuation_vector, force_vector):
    field = empty_field()

    y_step = (MAX_Y - MIN_Y) / Y_STEPS
    z_step = (MAX_Z - MIN_Z) / Z_STEPS

    for r in range(Z_STEPS):
        for c in range(Y_STEPS):
            cur_y = MIN_Y + c * y_step
            cur_z = MIN_Z + r * z_step

            # are we within rectangle?
            if min(y_from, y_to) <= cur_y <= max(y_from, y_to) and min(z_from, z_to) <= cur_z <= max(z_from, z_to):
                # calculate attenuation factor
                if attenuation_vector[0] != 0 or attenuation_vector[1] != 0:
                    y_ratio = abs((y_to - cur_y) / abs(y_to - y_from))
                    z_ratio = abs((z_to - cur_z) / abs(z_to - z_from))

                    aggr_ratio = y_ratio * attenuation_vector[0] + z_ratio * attenuation_vector[1]
                else:
                    aggr_ratio = 1

                field[r][c] = scale_vector((force_vector[0], force_vector[1]), aggr_ratio)

            else:
                field[r][c] = 0, 0

    return field


def constant_field(force_vector):
    field = empty_field()

    for r in range(Z_STEPS):
        for c in range(Y_STEPS):
            field[r][c] = force_vector[0], force_vector[1]

    return field


def elliptic_field(y_center, y_width, z_center, z_height, force_vector):
    field = empty_field()

    y_step = (MAX_Y - MIN_Y) / Y_STEPS
    z_step = (MAX_Z - MIN_Z) / Z_STEPS

    for r in range(Z_STEPS):
        for c in range(Y_STEPS):
            cur_y = MIN_Y + c * y_step
            cur_z = MIN_Z + r * z_step

            cur_y_relative = cur_y - y_center
            cur_z_relative = cur_z - z_center

            border_point = ellipse_border_point(y_width, z_height, (cur_y_relative, cur_z_relative))

            distance_to_border = sqrt(border_point[0] ** 2 + border_point[1] ** 2)
            distance_to_cur = sqrt(cur_y_relative ** 2 + cur_z_relative ** 2)

            # are we within ellipse?
            if distance_to_border >= distance_to_cur:
                # calculate attenuation factor
                ratio = 1 - distance_to_cur / distance_to_border
                field[r][c] = scale_vector((force_vector[0], force_vector[1]), ratio)
            else:
                field[r][c] = 0, 0

    return field


def radial_field(y_center, y_width, z_center, z_height, max_force):
    field = empty_field()

    y_step = (MAX_Y - MIN_Y) / Y_STEPS
    z_step = (MAX_Z - MIN_Z) / Z_STEPS

    for r in range(Z_STEPS):
        for c in range(Y_STEPS):
            cur_y = MIN_Y + c * y_step
            cur_z = MIN_Z + r * z_step

            cur_y_relative = cur_y - y_center
            cur_z_relative = cur_z - z_center

            border_point = ellipse_border_point(y_width, z_height, (cur_y_relative, cur_z_relative))

            distance_to_border = sqrt(border_point[0] ** 2 + border_point[1] ** 2)
            distance_to_cur = sqrt(cur_y_relative ** 2 + cur_z_relative ** 2)

            # are we within ellipse?
            if distance_to_border >= distance_to_cur:
                # calculate attenuation factor
                ratio = 1 - distance_to_cur / distance_to_border

                length = max_force * ratio
                angle_rad = atan(cur_y_relative / cur_z_relative)

                if cur_z_relative == 0:
                    y = 0
                    z = length
                else:
                    y = length * sin(angle_rad)
                    z = length * cos(angle_rad)

                if cur_z_relative < 0:
                    z *= -1
                    y *= -1

                field[r][c] = y, z

            else:
                field[r][c] = 0, 0

    return field

# get all fields filled at initialization
fill_fields()