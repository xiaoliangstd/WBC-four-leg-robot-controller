import math
import sys
import copy
SWING = 0
STANCE = 1
  # A swing leg that collides with the ground.
EARLY_CONTACT = 2
  # A stance leg that loses contact.
LOSE_CONTACT = 3

_STANCE_DURATION_SECONDS = [0.3] * 4
_DUTY_FACTOR = [0.6] * 4
_INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]


_INIT_LEG_STATE = [
    SWING,
    STANCE,
    STANCE,
    SWING,
]

class GaitGenerator:
    def __init__(self, robot):
        self._robot = robot
        self._stance_duration = copy.deepcopy(_STANCE_DURATION_SECONDS)
        self._duty_cycle = copy.deepcopy(_DUTY_FACTOR)
        self._initial_leg_phase = copy.deepcopy(_INIT_PHASE_FULL_CYCLE)
        self._initial_state_ratio_in_cycle = [0.0, 0.0, 0.0, 0.0]
        self._init_leg_state = copy.deepcopy(_INIT_LEG_STATE)
        self._next_leg_state = copy.deepcopy(_INIT_LEG_STATE)
        self._desired_leg_state = copy.deepcopy(_INIT_LEG_STATE)
        self._leg_state = copy.deepcopy(_INIT_LEG_STATE)

        self._normalized_phase = [0.0]*4

        self._contact_detection_phase_threshold = 0.1

        for leg_id in range(4):
            if self._init_leg_state[leg_id] == STANCE:
                self._initial_state_ratio_in_cycle[leg_id] = self._duty_cycle[leg_id]
                self._next_leg_state[leg_id] = SWING
            elif self._init_leg_state[leg_id] == SWING:
                self._initial_state_ratio_in_cycle[leg_id] = 1.0 - self._duty_cycle[leg_id]
                self._next_leg_state[leg_id] = STANCE







    def run(self, current_time):
        leg_contact_state = self._robot.getFootContactState()



        for leg_id in range(4):
            full_cycle_period = (self._stance_duration[leg_id] /
                                 self._duty_cycle[leg_id])


            augmented_time = current_time + self._initial_leg_phase[
                leg_id] * full_cycle_period
            phase_in_full_cycle = math.fmod(augmented_time,
                                            full_cycle_period) / full_cycle_period

            ratio = self._initial_state_ratio_in_cycle[leg_id]
            if phase_in_full_cycle < ratio:
                self._desired_leg_state[leg_id] = self._init_leg_state[leg_id]
                self._normalized_phase[leg_id] = phase_in_full_cycle / ratio
            else:
                # A phase switch happens for this leg.
                self._desired_leg_state[leg_id] = self._next_leg_state[leg_id]
                self._normalized_phase[leg_id] = (phase_in_full_cycle -
                                                  ratio) / (1 - ratio)

            self._leg_state[leg_id] = self._desired_leg_state[leg_id]


            if (self._normalized_phase[leg_id] <
                    self._contact_detection_phase_threshold):
                continue

            if (self._leg_state[leg_id] == SWING
                    and leg_contact_state[leg_id]):
                self._leg_state[leg_id] = EARLY_CONTACT
            if (self._leg_state[leg_id] == STANCE
                    and not leg_contact_state[leg_id]):
                self._leg_state[leg_id] = LOSE_CONTACT






