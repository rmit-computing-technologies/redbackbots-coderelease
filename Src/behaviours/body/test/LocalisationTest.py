import os
import numpy as np
from enum import Enum
from math import radians
from BehaviourTask import BehaviourTask
from body.skills.Localise import Localise
from body.skills.Stand import Stand
from body.skills.WalkToPoint import WalkToPoint
from body.skills.Walk import Walk
from head.HeadAware import HeadAware
from util import log
from util.Timer import Timer
from util.Global import myHeadingUncertainty, myPosUncertainty

from util.FieldGeometry import (
    FIELD_CENTER,
    MIDDLE_TOP,
    MIDDLE_BOTTOM,

    ENEMY_LEFT_CORNER,
    ENEMY_LEFT_GOAL_BOX_CORNER,
    ENEMY_LEFT_PENALTY_BOX_CORNER,
    ENEMY_GOAL_CENTER,
    ENEMY_PENALTY_CENTER,
    ENEMY_RIGHT_GOAL_BOX_CORNER,
    ENEMY_RIGHT_PENALTY_BOX_CORNER,
    ENEMY_RIGHT_CORNER,

    OUR_RIGHT_CORNER,
    OUR_RIGHT_GOAL_BOX_CORNER,
    OUR_RIGHT_PENALTY_BOX_CORNER,
    OUR_GOAL_CENTRE,
    OUR_PENALTY_CENTER,
    OUR_LEFT_GOAL_BOX_CORNER,
    OUR_LEFT_PENALTY_BOX_CORNER,
    OUR_LEFT_CORNER,

    heading_error
)


class Vertices(Enum):
    """
    Enum required to get both the name of vertex and value.
    """
    FIELD_CENTER = FIELD_CENTER
    MIDDLE_TOP = MIDDLE_TOP

    # opponent side of field
    ENEMY_RIGHT_CORNER = ENEMY_RIGHT_CORNER
    ENEMY_RIGHT_PENALTY_BOX_CORNER = ENEMY_RIGHT_PENALTY_BOX_CORNER
    ENEMY_GOAL_CENTER = ENEMY_GOAL_CENTER
    ENEMY_PENALTY_CENTER = ENEMY_PENALTY_CENTER
    ENEMY_LEFT_PENALTY_BOX_CORNER = ENEMY_LEFT_PENALTY_BOX_CORNER
    ENEMY_LEFT_CORNER = ENEMY_LEFT_CORNER

    MIDDLE_BOTTOM = MIDDLE_BOTTOM

    # our side of field
    OUR_RIGHT_CORNER = OUR_RIGHT_CORNER
    OUR_RIGHT_PENALTY_BOX_CORNER = OUR_RIGHT_PENALTY_BOX_CORNER
    OUR_GOAL_CENTRE = OUR_GOAL_CENTRE
    OUR_PENALTY_CENTER = OUR_PENALTY_CENTER
    OUR_LEFT_PENALTY_BOX_CORNER = OUR_LEFT_PENALTY_BOX_CORNER
    OUR_LEFT_CORNER = OUR_LEFT_CORNER


class LocalisationTest(BehaviourTask):
    """
    Walk to each vertex on the field.
    """
    TURN_RATE = 1.5
    HEADING_ERROR = radians(30)
    TIME_COVERSION = 1_000_000
    
    _localise_timer = Timer(10 * TIME_COVERSION)
    _stand_timer = Timer(3 * TIME_COVERSION)
    _walk_and_turn_timer = Timer()
    _turn_timer = Timer()

    _heading_check_vertex = FIELD_CENTER  #? potentially face two locations ahead? just for variation
    _test_idx = 0
    _vertices = list(Vertices)

    _metrics = []  # list of tuples eg (position reached, time taken, uncertainty)

    def _initialise_sub_tasks(self):        
        # Set sub-tasks
        self._sub_tasks = {
            "Localise": Localise(self),
            "WalkToNextVertex": WalkToPoint(self),
            "FacePoint": Walk(self),  # for turning
            "Stand": Stand(self),
        }


    def _transition(self):       
        # stay localising for the duration of _localise_timer
        if not self._localise_timer.finished():
            self._current_sub_task = "Localise"
            log.debug(f"Localisation time elapsed: {self._localise_timer.elapsed()/self.TIME_COVERSION:.2f}")
            return


        # if the localisation is lost, localise
        if HeadAware.high_uncertainty.is_max():
            self._current_sub_task = "Localise"
            return


        # handle transitions from WalkToNextVertex
        if self._current_sub_task == "WalkToNextVertex":
            # keep walking while not close to the target
            if not self._sub_tasks[self._current_sub_task]._position_close:
                # log.debug(f"Walking to vertex: {self._vertices[self._test_idx].name}")
                return

            # once arrived at the target vertex, log the position reached 
            # and _walking_timer time elapsed, and transition to FacePoint
            else:
                # self._log_pos_and_time()
                self._current_sub_task = "FacePoint"
                self._turn_timer.restart().start()
                return
            

        # handle transitions from FacePoint
        if self._current_sub_task == "FacePoint":
            # turn until facing the vertex
            if abs(heading_error(self._heading_check_vertex)) > self.HEADING_ERROR:
                # log.debug(f"Turning to face specified vertex: {self._heading_check_vertex}")
                return
            
            # if facing the vertex, proceed to Stand for duration of _stand_timer
            else:
                log.debug("Facing vertex. Starting stand timer.")
                self._current_sub_task = "Stand"
                self._log_pos_and_time()
                self._stand_timer.restart().start()
                return


        # handle transitions from Stand
        if self._current_sub_task == "Stand":
            # wait until _stand_timer has finished
            if not self._stand_timer.finished() and self._stand_timer.running:
                # log.debug(f"Stand time elapsed: {self._stand_timer.elapsed()/self.TIME_COVERSION:.2f}")
                return
            
            # after the _stand_timer has finished, set the next target vertex, 
            # transition to WalkToNextVertex and start the _walk_and_turn_timer
            else:
                log.debug("Stand time finished. Walking to next vertex.")
                if self._test_idx < len(self._vertices) - 1:
                    self._test_idx += 1  # move to the next vertex
                else:
                    self._test_idx = 0  # reset back to the first vertex
                    self._export_metrics()  # export metrics when restarting the loop

                # transition to walking and start the timer
                self._current_sub_task = "WalkToNextVertex"
                self._walk_and_turn_timer.restart().start()


        # entry point opens by walking to first vertex and start _walk_and_turn_timer
        if self._current_sub_task != "WalkToNextVertex":
            self._current_sub_task = "WalkToNextVertex"
            self._walk_and_turn_timer.restart().start()


    def _reset(self):
        self._current_sub_task = "Localise"
        self._localise_timer.restart().start()
        log.debug("Localisation timer started.")


    def _tick(self):
        # walk to the next location in the suite 
        if self._current_sub_task == "WalkToNextVertex":
            target = self._vertices[self._test_idx].value
            self._tick_sub_task(final_pos=target,
                                speed=0.2,
                                prevent_leaving_field=False)
            
        # face the center of the field
        elif self._current_sub_task == "FacePoint":
            self._tick_sub_task(turn=self.TURN_RATE if heading_error(self._heading_check_vertex) > 0
                else -self.TURN_RATE)
        else:
            self._tick_sub_task()

            
    def _log_pos_and_time(self):
        # get the location of starting point target point, and time elapsed for leg of journey
        prev_vertex = self._vertices[self._test_idx-1]
        prev_vertex.value.x, prev_vertex.value.y = \
            round(prev_vertex.value.x), round(prev_vertex.value.y)

        curr_vertex = self._vertices[self._test_idx]
        curr_vertex.value.x, curr_vertex.value.y = \
            round(curr_vertex.value.x), round(curr_vertex.value.y)

        distance = round(prev_vertex.value.distanceTo(curr_vertex.value), 2)

        # format the elapsed time for total walk_and_turn_time, + turn_time and walk_time separately
        walk_and_turn_time = round(self._walk_and_turn_timer.elapsed()/self.TIME_COVERSION, 2)
        turn_time = round(self._turn_timer.elapsed()/self.TIME_COVERSION, 2)
        walk_time = round(walk_and_turn_time - turn_time, 2)

        # record position and heading uncertainties
        # exact definitions of unit can be found in CMKF.cpp
        pos_uncertainty = round(myPosUncertainty(), 2)  # area of smallest uncertainty rectangle (lower = better)
        heading_uncertainty = round(myHeadingUncertainty(), 2)  # std dev of radians (lower = better)

        # log vertex reached and time taken for that leg of journey
        log.info(f"Arrived at {curr_vertex.name}", say=True)
        log.debug(f"Reached vertex {self._test_idx} [{curr_vertex.name}]: "\
                  f"({curr_vertex.value.x}, {curr_vertex.value.y}) "\
                  f"in {walk_time} seconds")
        log.debug(f"Distance travelled: {distance}mm")
        log.debug(f"Time to turn: {turn_time} seconds")
        log.info(f"Time elapsed between {prev_vertex.name} "\
                 f"and {curr_vertex.name} and face vertex: "\
                 f"{walk_and_turn_time} seconds.")
        
        log.debug(f"Position uncertainty: {pos_uncertainty}")
        log.debug(f"Heading uncertainty: {heading_uncertainty}")

        # add a tuple of metrics for current leg to _metrics
        self._metrics.append(
            (
                prev_vertex.name,    # pos from
                curr_vertex.name,    # pos to
                distance,            # distance walked
                walk_time,           # walk time
                turn_time,           # turn time
                walk_and_turn_time,  # total walk and turn time
                pos_uncertainty,     # position uncertainty (std dev of radians, lower = better)
                heading_uncertainty  # heading uncertainty (uncertainty rect area, lower = better)
            )
        )


    def _export_metrics(self):
        # format the metrics into a string for visualisation
        print(f"\n{'From':30} | {'To':30} | {'Distance':8} | {'Walk':5} | {'Turn':5} | {'Total':5} | {'Pos Uncert':10} | {'Head Uncert':11}")
        print(f"{'':-<31}|{'':-<32}|{'':-<10}|{'':-<7}|{'':-<7}|{'':-<7}|{'':-<12}|{'':-<12}")
        output = "".join(
            f"{metric[0]:30} | "     # pos from
            f"{metric[1]:30} | "     # pos to
            f"{metric[2]:8} | "      # distance walked
            f"{metric[3]:5} | "      # walk time
            f"{metric[4]:5} | "      # turn time
            f"{metric[5]:5} | "      # total walk and turn time
            f"{metric[6]:10} | "     # position uncertainty
            f"{metric[7]:11}\n"      # heading uncertainty
            for metric in self._metrics)
        print(output)

        # save the recorded metrics to csv
        self._save_metrics_to_csv(
            headers=["From", "To", "Distance (mm)", 
                     "Walk Time (s)", "Turn Time (s)", "Total Time (s)", 
                     "Position Uncertainty", "Heading Uncertainty"]
            )


    def _save_metrics_to_csv(self, path=os.path.join(os.environ["HOME"], "test"), headers=None):
        """Save the list of metrics to a csv file using numpy.
        
        Args:
            filename (str): Name of the output csv file
            headers (list): Optional list of column headers
        """
        # convert list of metrics to numpy array
        data = np.array(self._metrics)

        # create path if it doesn't exist
        os.makedirs(path, exist_ok=True)

        # establish the filename and path
        # could be separated by date and time. currently just overwrites the same file
        output_location = os.path.join(path, "localisation_test_metrics.csv")
        
        # if headers are provided, save with headers
        if headers:
            # format string for headers
            header_string = ",".join(headers)
            
            # save array to csv with headers and ensure each is formatted as a string
            np.savetxt(output_location, data, delimiter=",", 
                       header=header_string, fmt="%s", comments="")
        else:
            # save array to csv without headers
            np.savetxt(output_location, data, delimiter=",", fmt="%s")

        # get the file from the robot back to the laptop
        print("Retrieve the csv file...\n"\
              "  On Nao (SSH):\n"\
             f"    scp {output_location} <username>@<device_ip>:/path/on/your/device/\n"\
              "  On VM/PC:\n"\
             f"    scp nao@<robot_name/ip>:{output_location} /path/on/your/device/")
