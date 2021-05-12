#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import subprocess
import signal
import requests
import time
import json

api_server_url = 'http://localhost:8080/'

office_tasks = [
    {"task_type": "Loop", "start_time": 0, "description":
     {"start_name": "coe", "finish_name": "supplies", "num_loops": 1}},
    # TODO: Fix delivery not working
    {"task_type": "Delivery", "start_time": 0, "description":
     {"pickup_place_name": "pantry",
      "pickup_dispenser": "coke_dispenser",
      "dropoff_place_name": "hardware_2",
      "dropoff_ingestor": "coke_ingestor"}},
    {"task_type": "Loop", "start_time": 0, "description":
     {"start_name": "lounge", "finish_name": "coe", "num_loops": 1}}
]

airport_terminal_tasks = [
    {"task_type": "Clean", "start_time": 0, "priority": 0,
     "description": {"cleaning_zone": "zone_1"}},
    {"task_type": "Clean", "start_time": 0, "priority": 0,
     "description": {"cleaning_zone": "zone_4"}},
    {"task_type": "Loop", "start_time": 0, "priority": 1, "description":
        {"num_loops": 2, "start_name": "n23", "finish_name": "n24"}}
    # TODO: mop cart will drop when step size is high, thus wont test this
    # {"task_type": "Delivery", "start_time": 0, "description":
    #     {"pickup_place_name": "mopcart_pickup",
    #      "pickup_dispenser": "mopcart_dispenser",
    #      "dropoff_place_name": "spill",
    #      "dropoff_ingestor": "mopcart_collector"}}
]

clinic_tasks = [
    {"task_type": "Loop", "start_time": 0, "priority": 0, "description":
        {"num_loops": 1,
         "start_name": "L1_left_treatment_1",
         "finish_name": "L2_sub_waiting_area_1"}},
    {"task_type": "Loop", "start_time": 0, "priority": 0, "description":
        {"num_loops": 1,
         "start_name": "L1_left_nurse_center",
         "finish_name": "L2_south_counter"}},
    {"task_type": "Loop", "start_time": 1, "priority": 0, "description":
        {"num_loops": 1,
         "start_name": "L2_left_nurse_center",
         "finish_name": "L2_sub_waiting_area_2"}},
    {"task_type": "Loop", "start_time": 2, "priority": 0, "description":
        {"num_loops": 1,
         "start_name": "L2_north_counter",
         "finish_name": "L1_right_nurse_center"}}
]


class RMFSenarioTest:

    def __init__(
            self,
            world_name: str,
            total_robots: int,
            timeout_sec=50):
        """
        world_name arg:     Launch file world name
        total_robots arg:   number of robots in the world
        timeout_sec arg:    Time out for init
        """
        self.world_name = world_name
        launch_cmd = (f"ros2 launch rmf_demos {world_name}.launch.xml"
                      " headless:=1")
        print(f" Initialize command [{launch_cmd}]")
        self.proc1 = subprocess.Popen(launch_cmd,
                                    #   stdout=subprocess.DEVNULL,
                                      stderr=subprocess.DEVNULL,
                                      shell=True, preexec_fn=os.setsid)

        # Here we will check if the robot state is avail to determine whether
        # the rmf world has launched successfully.
        # Periodically Checks the tasks till timeout
        start = time.time()
        robot_states_res = None
        while True:
            if ((time.time() - start) > timeout_sec):
                raise RuntimeError(f"TimeOut in launching: {launch_cmd}, "
                                   f"Get request, res: {robot_states_res}")
            time.sleep(2)
            try:
                r = requests.get(url=api_server_url + "robot_list")
            except requests.exceptions.ConnectionError:
                continue
            if (r.status_code != 200):
                continue

            robot_states_res = r.json()
            if (len(robot_states_res) == total_robots):
                break

        # Warm up
        print(f"World is ready, received status from {total_robots} robots")
        time.sleep(2)

        # This is to speed up the simulation
        self.proc2 = subprocess.Popen("gz physics -s 0.01", shell=True)
        time.sleep(4)

        # Check if intermediate command is success
        self.proc1.poll()
        self.proc2.poll()
        print(f"kill: {self.proc1.returncode} {self.proc2.returncode}")

        if (self.proc2.returncode != 0):  # 0 means good
            raise RuntimeError("gz physics -s: Command Error")

    def __del__(self):
        print(f"Destructor is called! <Kill {self.world_name}>\n")
        self.stop()

    def stop(self):
        # Send the signal to all the process groups in gazebo launch
        os.killpg(os.getpgid(self.proc1.pid), signal.SIGTERM)
        self.proc2.kill()
        self.proc1.poll()
        self.proc2.poll()

    def start(
            self,
            task_requests: list,
            timeout_sec: int):
        """
        This will start the intergration test by sending multiple task
        requests to rmf. Return True if all tasks passed, else false.

        TODO: should use ros_time as timeout
        """
        print(f"Start senario with {len(task_requests)} requests")
        for req in task_requests:
            print(" - request a task: ", req)
            r = requests.post(api_server_url + 'submit_task', json=req)
            time.sleep(1)
            if (r.status_code != 200):
                print("Failed to connect api-server: not 200")
                return False
            if (r.json == ""):
                print("Task Submission failed!", r.json())
                return False

        # Periodically Checks the tasks
        start = time.time()
        while ((time.time() - start) < timeout_sec):
            time.sleep(3)
            r = requests.get(url=api_server_url + "task_list")

            if (r.status_code != 200):
                print("Not able to get tasklist from api-server")
                return

            success_count = 0
            for task in r.json():
                print(f"task: {task['task_id']} \t "
                      f"| robot: {task['robot_name']} \t"
                      f"| state: {task['state']} ")
                if (task['state'] == 'Completed'):
                    success_count += 1
                if (task['state'] == 'Failed'):
                    return False
            print("------------"*5)
            if success_count == len(task_requests):
                print(f"[{self.world_name}] All {success_count} Tasks Passed")
                return True
        return False


###############################################################################


def main(args=None):
    print("Starting integration test")

    ###########################################################################
    # Test Senario 1: Office World with 3 requests

    office = RMFSenarioTest("office", 2)
    success = office.start(office_tasks, 150)
    office.stop()
    del office

    if not success:
        raise RuntimeError

    # ###########################################################################
    # # Test Senario 2: Airport World with 3 requests

    airport = RMFSenarioTest("airport_terminal", 11)
    success = airport.start(airport_terminal_tasks, 250)
    airport.stop()
    del airport

    if not success:
        raise RuntimeError

    # ###########################################################################
    # # Test Senario 3: Clinic World with 4 requests

    clinic = RMFSenarioTest("clinic", 4)
    success = clinic.start(clinic_tasks, 600)
    clinic.stop()
    del clinic

    if not success:
        raise RuntimeError

    print("====================== Successfully End All ======================")


if __name__ == "__main__":
    main(sys.argv)