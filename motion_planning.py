import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import re

# from planning_utils_starter import a_star, heuristic, create_grid
from planning_utils import a_star, heuristic, create_grid, closest_point, create_graph_voronoi, create_graph_probabilistic, get_min_north_east
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, options):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True

        self.goal_position = list(map(lambda s: float(s), str.split(options["goal_position"], ",")))

        self.probabilistic_mode = bool(options["probabilistic_mode"])
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # band_distance for all the waypoints except the last can be higher
            band_distance = 1.0
            if(len(self.waypoints) > 1):
                band_distance = 4.0
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < band_distance:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                print("velocity", np.linalg.norm(self.local_velocity[1:2]))
                print("velocity", np.linalg.norm(self.local_velocity[0:2]))
                if np.linalg.norm(self.local_velocity[1:2]) < 1.0:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1],
                          self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = int(self.goal_position[2] * 1.1) + 5
        SAFETY_DISTANCE = 5

        # Assuming home is always set at 0
        self.target_position[2] = self.goal_position[2]

        init_pos = self.get_home_coordinates()
        self.set_home_position(init_pos[1], init_pos[0], 0)

        local_position = global_to_local(self.global_position, self.global_home)
        goal_position = global_to_local(self.goal_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        graph = None
        north_offset = None
        east_offset = None
        alt_dim = False
        if(self.probabilistic_mode):
            alt_dim = True
            print('********************************************')
            print(" Using probabalistic mode")
            print('********************************************')
            t0 = time.time()
            graph = create_graph_probabilistic(data, TARGET_ALTITUDE, int(SAFETY_DISTANCE / 2))
            print('graph took {0} seconds to build'.format(time.time() - t0))
            north_offset, east_offset = get_min_north_east(data)
        else:
            print('********************************************')
            print(" Using graph with voronoi")
            print('********************************************')
            graph, north_offset, east_offset = create_graph_voronoi(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print(north_offset, east_offset)
        graph_start = self.get_relative_coords(local_position, north_offset, east_offset, alt_dim)
        graph_goal = self.get_relative_coords(goal_position, north_offset, east_offset, alt_dim)

        # Get closest points on the graph
        graph_start_c = closest_point(graph, graph_start)
        graph_goal_c = closest_point(graph, graph_goal)
        # Create Graph

        # Run A* to find a path from start to goal 
        print('Local Start and Goal and closest points: ', graph_start, graph_start_c, graph_goal, graph_goal_c)
        path, _ = a_star(graph, heuristic, graph_start_c, graph_goal_c)

        # Convert path to waypoints
        waypoints = [
            [
                graph_start[0] + north_offset, 
                graph_start[1] + east_offset, 
                TARGET_ALTITUDE, 
                0
            ]
        ] 
        waypoints = waypoints + [[
            int(np.ceil(p[0] + north_offset)),
            int(np.ceil(p[1] + east_offset)), 
            TARGET_ALTITUDE, 
            0] for p in path]

        waypoints = waypoints + [
            [
                int(np.ceil(graph_goal[0] + north_offset)), 
                int(np.ceil(graph_goal[1] + east_offset)), 
                TARGET_ALTITUDE, 
                0
            ]
        ]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

    def get_home_coordinates(self):
        line = re.split(",|lat0|lon0| |\n", open("colliders.csv").readline())
        line = list(filter(lambda l: l, line))
        return tuple(map(lambda a: float(a), line))

    def get_relative_coords(self, local_position, north_offset, east_offset, alt_dim=False):
        north = int(np.ceil(local_position[0] - north_offset))
        east = int(np.ceil(local_position[1] - east_offset))
        if alt_dim:
            alt = int(np.ceil(local_position[2]))
            return (north, east, alt)
        return (north, east)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")

    # sample positions
    # positions = [
    #     (-122.398157, 37.792474, 0),  # short
    #     (-122.396826, 37.794126, 3),  # short on building
    #     ####
    #     (-122.398917, 37.792579, 0),  # easy two turns, 20
    #     (-122.398692, 37.792685, 12),  # easy two turns + top of building
    #     (-122.399280, 37.792970, 80),  # easy two turns + crazy altitude
    #     (-122.398248, 37.796342, 0)  # Far
    # ]
    parser.add_argument('--goal_position', type=str, default="-122.398157,37.792474,0",
                        help="Lon, Lat, Alt: -122.398157,37.792474,0")
    parser.add_argument('--probabilistic_mode', type=bool, default=False, help="Enable probabilistic mode")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, {
        "goal_position": args.goal_position,
        "probabilistic_mode": args.probabilistic_mode
    })
    time.sleep(1)

    drone.start()
