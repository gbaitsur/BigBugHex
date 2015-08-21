#! /usr/bin/env python

# BigBug hexapod body description
from collections import namedtuple
from math import sqrt, degrees as d, radians as r, atan2

from bb_cs import CoordinateSystem, Position
from bb_utility import angle_ab, angle_ac, exp_r, sind, cosd, third_side, mm_to_m as m

MountDescriptor = namedtuple('MountDescriptor', 'x y z rotz')  # describes attachment point for a leg in Thorax cs
StoredAngles = namedtuple('StoredAngles', 'coxa femur tibia tarsus')

# noinspection PyTypeChecker
g_odom_cs = CoordinateSystem(None, "odom", 0, 0, 0, 0, 0, 0)


# noinspection PyBroadException
class Thorax(object):
    def __init__(self):
        self.cs = CoordinateSystem(g_odom_cs, "thorax", 0, 0, 0, 0, 0, 0)

        self.legs = dict()
        self.fill_legs()

    def fill_legs(self):
        # initializes legs and puts them in a dictionary

        # leg mount point is on the top surface of thorax
        self.legs["rf"] = Leg("rf", self, MountDescriptor(m(182.343), m(-104.843), m(0), r(-45)), (101, 102, 103, 19))
        self.legs["rm"] = Leg("rm", self, MountDescriptor(m(0), m(-114.198), m(0), r(-90)), (104, 105, 106, 11))
        self.legs["rr"] = Leg("rr", self, MountDescriptor(m(-182.343), m(-84.843), m(0), r(-135)), (107, 108, 109, 14))
        self.legs["lf"] = Leg("lf", self, MountDescriptor(m(182.343), m(104.843), m(0), r(45)), (116, 117, 118, 13))
        self.legs["lm"] = Leg("lm", self, MountDescriptor(m(0), m(114.198), m(0), r(90)), (113, 114, 115, 20))
        self.legs["lr"] = Leg("lr", self, MountDescriptor(m(182.343), m(84.843), m(0), r(135)), (110, 111, 112, 15))

        self.legs["rf"].rostral_neighbor = self.legs["lf"]
        self.legs["rf"].caudal_neighbor = self.legs["rm"]
        self.legs["rm"].rostral_neighbor = self.legs["rf"]
        self.legs["rm"].caudal_neighbor = self.legs["rr"]
        self.legs["rr"].rostral_neighbor = self.legs["rm"]
        self.legs["rr"].caudal_neighbor = self.legs["lr"]

        self.legs["lf"].rostral_neighbor = self.legs["rf"]
        self.legs["lf"].caudal_neighbor = self.legs["lm"]
        self.legs["lm"].rostral_neighbor = self.legs["lf"]
        self.legs["lm"].caudal_neighbor = self.legs["lr"]
        self.legs["lr"].rostral_neighbor = self.legs["lm"]
        self.legs["lr"].caudal_neighbor = self.legs["rr"]

    # noinspection PyPep8Naming
    @property
    def ground_clearance(self):
        # returns ground clearance based on current leg positions

        # legs are attached to a top surface of thorax
        # as leg Z in thorax cs is negative when foot is below thorax, ground clearance is assumed to be (max leg Z) - (thorax height)
        THORAX_HEIGHT = m(48)

        z_list = []
        for leg in self.legs.values():
            if not leg.in_swing:  # only legs in stance are considered
                z_list.append(leg.get_node("foot-thorax").z)  # get coordinates of all feet in thorax cs
        if len(z_list) > 0:
            return -1 * max(z_list) - THORAX_HEIGHT
        else:
            return 0  # this generally should not happen, unless hex is flying

    @property
    def r_stance(self):
        # returns maximum restrictedness of all legs in stance
        r_max = 0
        for leg in self.legs.values():
            if not leg.in_swing:
                r_max = max(r_max, leg.r_stance)
        return r_max

    def move(self, keep_feet_odom=False, new_x=None, new_y=None, new_z=None, new_rotx=None, new_roty=None, new_rotz=None):
        # moves thorax
        # keep_feet_odom = true means that feet must stay where they are in odom cs and not move together with thorax
        self.cs.redefine(new_x, new_y, new_z, new_rotx, new_roty, new_rotz)
        for leg in self.legs.values():
            leg._r_ws_inner = None
            leg._r_ws_outer = None
            if not keep_feet_odom:
                try:
                    del leg.stored_node_positions["foot-odom"]  # feet shift together with thorax, so their odom coordinates must be updated
                except:
                    pass  # feet remain where they are, so no need to update their odom coordinates

    def rollback(self, keep_feet_odom):
        # moves thorax back to a stored position
        # keep_feet_odom = true means that feet must stay where they are in odom cs and not move together with thorax
        self.cs.rollback()

        for leg in self.legs.values():
            leg._r_ws_inner = None
            leg._r_ws_outer = None
            if not keep_feet_odom:
                try:
                    del leg.stored_node_positions["foot-odom"]  # feet shift together with thorax, so their odom coordinates must be updated
                except:
                    pass  # feet remain where they are, so no need to update their odom coordinates


# noinspection PyBroadException
class Leg(object):
    COXA_LENGTH = m(42.8)
    FEMUR_LENGTH = m(70)
    TIBIA_LENGTH = m(67.5)
    TARSUS_LENGTH = m(71.3)
    INNER_WS_LIMIT = m(COXA_LENGTH + 70)
    OUTER_WS_LIMIT = m(165)

    def __init__(self, name, parent, mount_descriptor, servo_ids):
        global COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH
        
        self.name = name
        self.parent = parent
        self.mount_descriptor = mount_descriptor

        self.swing_order = 0  # used to keep track of the order in which legs swing. If two legs are ready to swing, priority is given to the one that has been in stance longer

        self.r_was_increasing = False  # indicates whether restrictedness of this leg was increasing before reaching flat plateau

        self.cs = CoordinateSystem(parent.cs, self.name, mount_descriptor.x, mount_descriptor.y, mount_descriptor.z, 0, 0, mount_descriptor.rotz)

        self.segments = dict()
        self.segments["coxa"] = Segment(name=name + "_coxa", servo_id=servo_ids[0],
                                        min_angle=r(-35), neutral_angle=r(0), max_angle=r(35),
                                        length=COXA_LENGTH, parent=self,
                                        leg=self,
                                        offset_in_parent=m(0), mirrored=(mount_descriptor.x < 0), z_rot=True)

        if self.name == "rf":
            self.segments["coxa"].max_angle = r(10)
        elif self.name == "lf":
            self.segments["coxa"].min_angle = r(-10)
        elif self.name == "rr":
            self.segments["coxa"].min_angle = r(-10)
        elif self.name == "lr":
            self.segments["coxa"].max_angle = r(10)

        self.segments["femur"] = Segment(name=name + "_femur", servo_id=servo_ids[1],
                                         min_angle=r(-45), neutral_angle=r(20), max_angle=r(90), length=FEMUR_LENGTH,
                                         parent=self.segments["coxa"], leg=self,
                                         offset_in_parent=COXA_LENGTH)

        self.segments["tibia"] = Segment(name=name + "_tibia", servo_id=servo_ids[2],
                                         min_angle=r(-115), neutral_angle=r(20), max_angle=r(20), length=TIBIA_LENGTH,
                                         parent=self.segments["femur"], leg=self,
                                         offset_in_parent=FEMUR_LENGTH)

        self.segments["tarsus"] = Segment(name=name + "_tarsus", servo_id=servo_ids[3],
                                          min_angle=r(-90), neutral_angle=r(0), max_angle=r(0), length=TARSUS_LENGTH,
                                          parent=self.segments["tibia"], leg=self,
                                          offset_in_parent=TIBIA_LENGTH)

        self.rostral_neighbor = None  # leg in front
        self.caudal_neighbor = None  # leg behind

        self.in_swing = False  # is this leg in swing

        self.swing_z = 0  # target highest Z of current swing (calculated as Z of lift-off point + DEFAULT_SWING_HEIGHT), all in thorax cs

        self.touched_down = False  # whether the leg actually touches ground

        self.wants_to_swing = False  # previously the leg wanted to swing, but was not allowed to do it

        self.landing = False  # leg is forced to land
        self.raising = False  # leg is in initial phase of swing

        coxa_seg = self.segments["coxa"]
        self.home_angle = (coxa_seg.min_angle + coxa_seg.max_angle) / 2
        self.home_position_leg = Position(self.cs)
        self.home_position_thorax = Position(self.cs.parent)

        self.task_cs = CoordinateSystem(parent.cs, self.name + "_tcs", 0, 0, 0, 0, 0, 0)

        self.swings_inwards = False  # foot swings towards thorax

        self._r_ws_inner = None  # restrictedness by inner edge of foot workspace
        self._r_ws_outer = None  # restrictedness by outer edge of foot workspace

        # stored previous values of restrictedness components
        self._r_coxa_prev = 0
        self._r_femur_prev = 0
        self._r_tibia_prev = 0
        self._r_tarsus_prev = 0
        self._r_stretch_prev = 0
        self._r_contract_prev = 0
        self._r_ws_outer_prev = 0
        self._r_ws_inner_prev = 0

        self._r_prev = 0  # previous total restrictedness

        # back-ups for angles and leg node positions
        self.angles_backup = StoredAngles(0, 0, 0, 0)
        self.nodes_backup = dict()

        self.stored_node_positions = dict()  # already calculated coordinates of leg nodes

        # coxa coordinates in leg cs and thorax cs never change
        self.stored_node_positions["coxa"] = Position(self.cs, 0, 0, 0)
        self.stored_node_positions["coxa-thorax"] = self.cs.parent.to_this(self.stored_node_positions["coxa"])

    def __str__(self):
        return self.name

    def update_swings_inwards(self):
        # Indicates whether the swing movement is towards inner limit of workspace

        coxa = self.get_node("coxa")
        current_foot = self.get_node("foot")
        moved_foot = current_foot.clone
        moved_foot.x += .0001 * sind(-1 * self.task_cs.rotz)
        moved_foot.y += .0001 * cosd(-1 * self.task_cs.rotz)

        self.swings_inwards = coxa.distance_to(current_foot) > coxa.distance_to(moved_foot)

    @property
    def joint_angles(self):
        # return joint angles as tuple
        segment_coxa = self.segments["coxa"]
        segment_femur = self.segments["femur"]
        segment_tibia = self.segments["tibia"]
        segment_tarsus = self.segments["tarsus"]

        coxa = segment_coxa.current_angle
        femur = segment_femur.current_angle
        tibia = segment_tibia.current_angle
        tarsus = segment_tarsus.current_angle

        return coxa, femur, tibia, tarsus

    def set_joint_angles(self, coxa, femur, tibia, tarsus):
        # set joint angles with tuple
        segment_coxa = self.segments["coxa"]
        segment_femur = self.segments["femur"]
        segment_tibia = self.segments["tibia"]
        segment_tarsus = self.segments["tarsus"]

        segment_coxa.current_angle = coxa
        segment_femur.current_angle = femur
        segment_tibia.current_angle = tibia
        segment_tarsus.current_angle = tarsus

        self.stored_node_positions.clear()

        self._r_ws_inner = None
        self._r_ws_outer = None

    def store_node(self, name):

        # noinspection PyShadowingNames
        def x_y(l, coxa_angle):
            x = l * sind(-coxa_angle)
            y = l * cosd(-coxa_angle)
            return x, y

        if name == "foot":
            segment_coxa = self.segments["coxa"]
            coxa_len = segment_coxa.length
            coxa_angle = segment_coxa.current_angle

            segment_femur = self.segments["femur"]
            femur_angle = segment_femur.current_angle
            femur_projection_h = segment_femur.length * cosd(femur_angle)
            femur_projection_v = segment_femur.length * sind(femur_angle)

            segment_tibia = self.segments["tibia"]
            tibia_angle = segment_tibia.current_angle
            tibia_projection_h = segment_tibia.length * cosd(femur_angle + tibia_angle)
            tibia_projection_v = segment_tibia.length * sind(femur_angle + tibia_angle)

            segment_tarsus = self.segments["tarsus"]
            tarsus_angle = segment_tarsus.current_angle
            tarsus_projection_h = segment_tarsus.length * sind(90 + (tibia_angle + femur_angle + tarsus_angle) - 180)
            tarsus_projection_v = segment_tarsus.length * cosd(90 + (tibia_angle + femur_angle + tarsus_angle) - 180)

            l = coxa_len + femur_projection_h + tibia_projection_h + tarsus_projection_h
            z = femur_projection_v + tibia_projection_v + tarsus_projection_v
            x, y = x_y(l, coxa_angle)
            node_pos_converted = Position(self.cs)
            node_pos_converted.x, node_pos_converted.y, node_pos_converted.z = x, y, z

        elif name == "foot-odom":
            foot_thorax = self.get_node("foot-thorax")
            node_pos_converted = self.cs.parent.parent.to_this(foot_thorax)

        elif name == "coxa-thorax":
            segment_coxa = self.segments["coxa"]
            node_pos = Position(segment_coxa.cs)
            node_pos_converted = self.cs.parent.to_this(node_pos)

        elif name == "foot-thorax":
            foot = self.get_node("foot")
            node_pos_converted = self.cs.parent.to_this(foot)

        elif name == "tibia-thorax":
            segment_coxa = self.segments["coxa"]
            coxa_len = segment_coxa.length
            coxa_angle = segment_coxa.current_angle

            segment_femur = self.segments["femur"]
            femur_angle = segment_femur.current_angle
            femur_projection_h = segment_femur.length * cosd(femur_angle)
            femur_projection_v = segment_femur.length * sind(femur_angle)

            l = coxa_len + femur_projection_h
            z = femur_projection_v
            x, y = x_y(l, coxa_angle)
            node_pos_converted = Position(self.cs)
            node_pos_converted.x, node_pos_converted.y, node_pos_converted.z = x, y, z

        else:
            node_pos = Position(self.segments[name].cs)
            node_pos_converted = self.cs.to_this(node_pos)

        self.stored_node_positions[name] = node_pos_converted

        return node_pos_converted

    def get_node(self, name):
        try:
            return self.stored_node_positions[name]
        except:
            return self.store_node(name)

    def store_foot_odom(self, foot_thorax=None):
        if foot_thorax is None:
            self.stored_node_positions["foot-odom"] = self.cs.parent.parent.to_this(self.get_node("foot"))
        else:
            self.stored_node_positions["foot-odom"] = foot_thorax.owner.parent.to_this(foot_thorax)

    def calc_joint_angles(self, target_position, best_effort):

        general_calculation = True

        position_leg_cs = self.cs.to_this(target_position)

        angle_coxa = -1 * d(atan2(position_leg_cs.x, position_leg_cs.y))
        femur_to_position_projection = 0

        femur_segment = self.segments["femur"]
        femur_length = femur_segment.length
        tibia_segment = self.segments["tibia"]
        tibia_length = tibia_segment.length
        tarsus_segment = self.segments["tarsus"]
        tarsus_length = tarsus_segment.length

        coxa_segment = self.segments["coxa"]

        if best_effort:
            tgt_tarsus = position_leg_cs.clone
            tgt_tarsus.z += tarsus_length

            femur = self.get_node("femur")

            # check if we can actually reach target_position
            distance = femur.distance_to(tgt_tarsus)

            max_dist = femur_length + tibia_length
            min_dist = third_side(femur_length, tibia_length, 180 + tibia_segment.min_angle)

            dz = femur.z - tgt_tarsus.z

            # are we too far?
            if distance > max_dist:
                # yep, too far
                # calculate femur angle to target
                femur_to_position_projection = sqrt(max_dist ** 2 - dz ** 2)
                general_calculation = False
            elif distance < min_dist:
                # we are too close
                femur_to_position_projection = sqrt(min_dist ** 2 - dz ** 2)
                general_calculation = False
            else:
                # we are not too close and not too far
                general_calculation = True

        if general_calculation:
            # works either when not best_effort or when best_effort and distance is ok
            coxa_to_position_projection = sqrt(position_leg_cs.x ** 2 + position_leg_cs.y ** 2)
            femur_to_position_projection = coxa_to_position_projection - coxa_segment.length

        distance_to_tarsus_top = sqrt(femur_to_position_projection ** 2 + (position_leg_cs.z + tarsus_length) ** 2)

        if position_leg_cs.z + tarsus_length == 0:
            angle_down_to_tarsus_top = 0
        else:
            angle_down_to_tarsus_top = 90 - angle_ab(-1 * (position_leg_cs.z + tarsus_length), distance_to_tarsus_top, femur_to_position_projection)

        angle_femur = angle_ab(femur_length, distance_to_tarsus_top, tibia_length) - angle_down_to_tarsus_top
        angle_tibia = -180 + angle_ac(femur_length, distance_to_tarsus_top, tibia_length)
        angle_tarsus = -180 + 90 - angle_femur - angle_tibia

        if coxa_segment.min_angle > angle_coxa:
            if best_effort:
                angle_coxa = coxa_segment.min_angle
            else:
                raise Exception(self.name + " coxa angle too low")
        elif coxa_segment.max_angle < angle_coxa:
            if best_effort:
                angle_coxa = coxa_segment.max_angle
            else:
                raise Exception(self.name + " coxa angle too high")

        if femur_segment.min_angle > angle_femur:
            if best_effort:
                angle_femur = femur_segment.min_angle
            else:
                raise Exception(self.name + " femur angle too low")
        elif femur_segment.max_angle < angle_femur:
            if best_effort:
                angle_femur = femur_segment.max_angle
            else:
                raise Exception(self.name + " femur angle too high")

        if tibia_segment.min_angle > angle_tibia:
            if best_effort:
                angle_tibia = tibia_segment.min_angle
            else:
                raise Exception(self.name + " tibia angle too low")
        elif tibia_segment.max_angle < angle_tibia:
            if best_effort:
                angle_tibia = tibia_segment.max_angle
            else:
                raise Exception(self.name + " tibia angle too high")

        if tarsus_segment.min_angle > angle_tarsus:
            if best_effort:
                angle_tarsus = tarsus_segment.min_angle
            else:
                raise Exception(self.name + " tarsus angle too low")
        elif tarsus_segment.max_angle < angle_tarsus:
            if best_effort:
                angle_tarsus = tarsus_segment.max_angle
            else:
                raise Exception(self.name + " tarsus angle too high")

        return angle_coxa, angle_femur, angle_tibia, angle_tarsus

    def backup(self):
        # stores current joint angles so that they could be restored
        self.angles_backup = StoredAngles(self.segments["coxa"].current_angle, self.segments["femur"].current_angle, self.segments["tibia"].current_angle, self.segments["tarsus"].current_angle)
        self.nodes_backup = dict()
        for node_name in self.stored_node_positions:
            self.nodes_backup[node_name] = self.stored_node_positions[node_name]

    def rollback(self):
        # restores joint angles to what is stored
        self.set_joint_angles(self.angles_backup.coxa, self.angles_backup.femur, self.angles_backup.tibia, self.angles_backup.tarsus)
        self.stored_node_positions = self.nodes_backup

    def update_home_position(self):
        h_dist_home = (Leg.OUTER_WS_LIMIT + Leg.INNER_WS_LIMIT) / 2
        roll_dist_home = h_dist_home * cosd(self.cs.roty)

        self.home_position_leg = Position(self.cs, roll_dist_home * sind(-self.home_angle), roll_dist_home * cosd(-self.home_angle), 0)
        self.home_position_leg.z = -38

        self.home_position_thorax = self.cs.parent.to_this(self.home_position_leg)

        return self.home_position_leg

    def redefine_task_cs(self, translate_x, translate_y, rotate_z):
        if not (translate_x == 0 and translate_y == 0 and rotate_z == 0):
            # calculate movement vector angle
            current_foot_pos_thorax = self.get_node("foot-thorax")
            odom_cs = self.cs.parent.parent  # leg->thorax->odom
            foot_pos_odom = odom_cs.to_this(current_foot_pos_thorax)
            moved_thorax_cs = self.cs.parent.get_derivative("tmp_thorax_for_tcs_vector", translate_x, translate_y, 0, 0, 0, rotate_z)
            new_foot_pos_thorax = moved_thorax_cs.to_this(foot_pos_odom, caller="redefine_task_cs")

            dx = new_foot_pos_thorax.x - current_foot_pos_thorax.x
            dy = new_foot_pos_thorax.y - current_foot_pos_thorax.y

            angle = d(atan2(dx, dy)) - 180

            home_pos = self.home_position_thorax

            self.task_cs.redefine_tuple(home_pos.tuple, (None, None, -angle))

            self.update_swings_inwards()
        else:
            self.swings_inwards = None

    @property
    def anterior_neighbor(self):
        angle = self.task_cs.rotz
        if -90 < angle < 90:
            return self.rostral_neighbor
        else:
            return self.caudal_neighbor

    @property
    def posterior_neighbor(self):
        angle = self.task_cs.rotz
        if -90 < angle < 90:
            return self.caudal_neighbor
        else:
            return self.rostral_neighbor

    @property
    def can_swing(self):
        return not self.caudal_neighbor.in_swing and not self.rostral_neighbor.in_swing

    # RESTRICTEDNESS -->

    def store_r(self):
        # store restrictedness components

        self._r_coxa_prev = self.r_coxa

        self._r_femur_prev = self.r_femur
        self._r_tibia_prev = self.r_tibia
        self._r_tarsus_prev = self.r_tarsus

        self._r_ws_inner_prev = self.r_ws_inner("store_r/")
        self._r_ws_outer_prev = self.r_ws_outer("store_r/")

        self._r_stretch_prev = self.r_stretch
        self._r_contract_prev = self.r_contract

        self._r_prev = self.r()

    @property
    def r_ws_inner(self):
        if self.r_ws_inner is None:
            self.update_r_ws()
        return self._r_ws_inner

    @property
    def r_ws_outer(self):
        if self.r_ws_outer is None:
            self.update_r_ws()
        return self._r_ws_outer

    @property
    def r_coxa(self):
        lower_limit = self.segments["coxa"].min_angle
        upper_limit = self.segments["coxa"].max_angle
        transition_zone = 20
        parameter = self.segments["coxa"].current_angle

        return exp_r(parameter, lower_limit, upper_limit, transition_zone)

    @property
    def r_femur(self):
        lower_limit = self.segments["femur"].min_angle
        upper_limit = self.segments["femur"].max_angle
        transition_zone = 10
        parameter = self.segments["femur"].current_angle

        return exp_r(parameter, lower_limit, upper_limit, transition_zone)

    @property
    def r_tibia(self):
        lower_limit = self.segments["tibia"].min_angle
        upper_limit = self.segments["tibia"].max_angle
        transition_zone = 10
        parameter = self.segments["tibia"].current_angle

        return exp_r(parameter, lower_limit, upper_limit, transition_zone)

    @property
    def r_tarsus(self):
        lower_limit = self.segments["tarsus"].min_angle
        upper_limit = self.segments["tarsus"].max_angle
        transition_zone = 10
        parameter = self.segments["tarsus"].current_angle

        return exp_r(parameter, lower_limit, upper_limit, transition_zone)

    def update_r_ws(self):
        # changes either when thorax CS is redefined (body moved) or when this leg is moved

        foot_position_odom = self.cs.parent.parent.to_this(self.get_node("foot-thorax"))  # leg->thorax->odom
        coxa_position_odom = self.cs.parent.parent.to_this(self.get_node("coxa-thorax"))  # leg->thorax->odom
        foot_position_odom.z = 0
        coxa_position_odom.z = 0

        transition_zone = 10

        parameter = coxa_position_odom.distance_to(foot_position_odom)

        lower_limit = Leg.INNER_WS_LIMIT
        upper_limit = 100000000
        self._r_ws_inner = exp_r(parameter, lower_limit, upper_limit, transition_zone)

        lower_limit = -100000000
        upper_limit = Leg.OUTER_WS_LIMIT
        self._r_ws_outer = exp_r(parameter, lower_limit, upper_limit, transition_zone)

    @property
    def r_ws(self):
        if self.in_swing:
            if self.swings_inwards:
                return self.r_ws_inner()
            else:
                return self.r_ws_outer()
        else:
            return self.r_ws_outer() + self.r_ws_inner()

    @property
    def r_stretch(self):
        lower_limit = -100000000
        upper_limit = (self.segments["femur"].length + self.segments["tibia"].length) * 0.9
        transition_zone = 5

        try:
            tarsus = self.stored_node_positions["tarsus"]
        except:
            tarsus = self.store_node("tarsus")

        try:
            femur = self.stored_node_positions["femur"]
        except:
            femur = self.store_node("femur")

        parameter = femur.distance_to(tarsus)

        return exp_r(parameter, lower_limit, upper_limit, transition_zone)

    @property
    def r_contract(self):
        lower_limit = third_side(self.segments["femur"].length, self.segments["tibia"].length, 180 + self.segments["tibia"].min_angle)
        upper_limit = 100000000
        transition_zone = 20

        try:
            tarsus = self.stored_node_positions["tarsus"]
        except:
            tarsus = self.store_node("tarsus")

        try:
            femur = self.stored_node_positions["femur"]
        except:
            femur = self.store_node("femur")

        parameter = femur.distance_to(tarsus)

        return exp_r(parameter, lower_limit, upper_limit, transition_zone)

    def r_stance(self):
        return max(self.r_coxa, self.r_femur, self.r_tibia, self.r_tarsus, self.r_ws)

    @property
    def r_swing(self):
        # in most cases growing r_coxa indicates final phase of swing
        # in a case when task_cs is exactly aligned with leg cs, r_coxa will be constant throughout swing, so alternative criterion is needed

        result = 0

        # is task_cs aligned with leg cs?
        tcs_angle = 0 + self.task_cs.rotz
        if tcs_angle > 180:
            tcs_angle -= 360

        if abs(tcs_angle - self.cs.rotz) < 0.0001:
            r_stretch = self.r_stretch
            if r_stretch > self._r_stretch_prev + 0.0001:
                result = max(self.r_tarsus, r_stretch)
        elif abs(tcs_angle + self.cs.rotz) < 0.0001:
            r_contract = self.r_contract
            if r_contract > self._r_contract_prev + 0.0001:
                result = max(self.r_tibia, r_contract)

        else:
            # use growing r_coxa as final swing indicator
            r_coxa = self.r_coxa
            if r_coxa > self._r_coxa_prev + 0.0001:
                r_stretch = self.r_stretch
                if r_stretch <= self._r_stretch_prev:
                    r_stretch = 0
                # r += max(r_coxa, self.r_tarsus, self.r_stretch)#, self.r_ws("r_swing"))
                if self.raising:
                    r_tarsus = 0
                else:
                    r_tarsus = self.r_tarsus

                result = max(r_coxa, r_tarsus, r_stretch)
            else:
                result = 0

        return result

    def r(self):
        if self.in_swing:
            return self.r_swing
        else:
            return self.r_stance()

    def r_change_direction(self, test_r=None):
        # returns 1 when restrictedness is increasing, 2 -- when decreasing, 0 -- when restrictedness is constant
        # if test_r is provided, previous restrictedness is compared to this test_r, otherwise -- to current leg restrictedness

        if test_r is None:
            new_r = self.r
        else:
            new_r = test_r

        if new_r > self._r_prev + 0.0001:
            return 1
        elif new_r < self._r_prev - 0.0001:
            return -1
        else:
            return 0

            # <-- RESTRICTEDNESS


class Segment(object):
    def __init__(self, name, servo_id, min_angle, neutral_angle, max_angle, length, parent, leg, offset_in_parent, mirrored=False, z_rot=False):
        self.name = name
        self.parent = parent
        self.leg = leg

        self.length = length
        self.offset_in_parent = offset_in_parent

        self.min_angle = min_angle
        self.max_angle = max_angle
        self.neutral_angle = neutral_angle

        if self.name == "lm_coxa":
            self.mount_correction = 45
        else:
            self.mount_correction = 0

        if "femur" in self.name:
            self.mount_correction = -90

        if "tibia" in self.name:
            self.load_direction = -1
        else:
            self.load_direction = 1

        self.mirrored = mirrored
        self.z_rot = z_rot
        self._current_angle = 0

        self.servo_id = servo_id

        self.lifted = False
        self.landing_counter = 0
        self.load = 0

        if self.z_rot:
            self.cs = CoordinateSystem(parent.cs, self.name, 0, self.offset_in_parent, 0, 0, 0, self._current_angle)
        else:
            self.cs = CoordinateSystem(parent.cs, self.name, 0, self.offset_in_parent, 0, self._current_angle, 0, 0)

    def __str__(self):
        return self.name

    @property
    def current_angle_sideaware(self):
        if self.mirrored:
            mirror_coef = -1
        else:
            mirror_coef = 1

        return self._current_angle * mirror_coef

    @property
    def current_angle(self):
        return self._current_angle

    @property
    def servo_angle(self):
        if "tibia" in self.name:
            reverse = -1
        else:
            reverse = 1

        if self.name[:1] == "l":
            side = -1 * reverse
        else:
            side = 1 * reverse

        return side * (self.current_angle_sideaware + self.neutral_angle + self.mount_correction)

    @current_angle.setter
    def current_angle(self, value):
        if "rr" in self.name and "test" not in self.name:
            pass
        if self.min_angle <= value <= self.max_angle:
            self._current_angle = value
        elif value < self.min_angle:
            self._current_angle = self.min_angle
        elif value < self.min_angle:
            self._current_angle = self.min_angle

        if self.z_rot:
            self.cs.redefine(new_rotz=self._current_angle)
        else:
            self.cs.redefine(new_rotx=self._current_angle)

    def set_angle_from_servo(self, angle):
        self._current_angle = self.angle_from_servo(angle)

    def angle_from_servo(self, angle):
        if "tibia" in self.name:
            reverse = -1
        else:
            reverse = 1

        if self.name[:1] == "l":
            side = -1 * reverse
        else:
            side = 1 * reverse

        if self.mirrored:
            mirror_coef = -1
        else:
            mirror_coef = 1

        angle = ((angle / side) - self.neutral_angle - self.mount_correction) / mirror_coef
        return angle