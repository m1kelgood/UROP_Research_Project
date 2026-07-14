import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
import random
import simple_route_planning as SRP
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from pathlib import Path
import os
import subprocess
import shutil
from datetime import datetime
import numpy as np



# ==============================================================================
# Global variables
# ==============================================================================
OSM_FILE = "../../osm/campus.osm"
JSON_OUTPUT_PATH = "../scenarios"

# this needs to match the geoReference line in the .xodr file
ORIGIN = Origin(50.785562924295, 6.04648796549892, 0)

ROAD_TYPES = [
    "roundabout", "urban", "one_way", "construction_site", "nonurban", "under_bridge"
]
MAX_ROUTE_LOOKUP = 20

MAP_BOUNDS = {
    "left": -400, "right": 330, "top": 750, "bottom": -750
}



# ==============================================================================
# Lanelet interface
# ==============================================================================
def find_all_roads(osm_file, ll_map, bounds=None):
    tree = ET.parse(osm_file)
    root = tree.getroot()

    ll_roads = []
    # for campus map: need the bounding box because not all of the osm map has been properly rendered in the xodr file
    if bounds is not None:
        lb = bounds.get("left")
        rb = bounds.get("right")
        tb = bounds.get("top")
        bb = bounds.get("bottom")

    for child in root:
        if child.tag == "relation":
            id = child.attrib.get("id")
            out_of_zone = []

            for grandchild in child:
                if grandchild.attrib.get("v") == "road":
                    lanelet = ll_map.laneletLayer[int(id)]
                    center = lanelet.centerline

                    for pt in center:
                        if pt.x < lb or pt.x > rb or pt.y > tb or pt.y < bb:
                            out_of_zone.append(id)
                            break
                    
                    if id not in out_of_zone:
                        ll_roads.append(id)

    return ll_roads


def find_certain_roads(osm_file, req_road_types):
    '''
    Can pass a list of ids as checkpoints and then force the vehicle to go on a route that consists of these checkpoints.
    To do this going to need to parse through the osm file and look for specifiv tags like <tag k="location" v="urban" />
    '''
    tree = ET.parse(osm_file)
    root = tree.getroot()

    ll_roads = set()
    for child in root:
        if child.tag == "relation":
            id = child.attrib.get("id")

            road_info = {}
            for grandchild in child:
                k = grandchild.attrib.get("k")
                v = grandchild.attrib.get("v")
                
                if k is not None and v is not None:
                    road_info[k] = v

            if "road" in list(road_info.values()):
                for key, val in road_info.items():
                    if key in req_road_types or val in req_road_types:
                        ll_roads.add(id)

    return list(ll_roads)


def stitch_route(routing_graph, all_roads, start, interm, end):
    '''
    Function to stitch three roads together due to the limitations of Lanelet's .getRoute method
    Doesn't work the best when a MAP_BOUNDING is applied. Need to modify the logic if further development is required.
    '''
    start_to_interm = routing_graph.getRoute(start, interm)
    interm_to_end = routing_graph.getRoute(interm, end)
    
    if start_to_interm is None or interm_to_end is None:
        return None
    
    total_route = []
    
    for r1 in start_to_interm.shortestPath():
        if r1 not in all_roads:
            return None
        total_route.append(r1)

    for r2 in interm_to_end.shortestPath():
        if r2 not in all_roads:
            return None
        total_route.append(r2)

    return total_route


def create_route(routing_graph, all_roads, ll_map, max_lookup=20, specific_roads=None, prioritize_short_len=True):
    routes = []

    shortest_route_len = float("inf")
    selected_route = None
    count = 0
    while count < max_lookup:
        start = int(random.choice(all_roads))
        end = int(random.choice(all_roads))

        ll_start = ll_map.laneletLayer[start]
        ll_end = ll_map.laneletLayer[end]

        if specific_roads is not None:
            interm = int(random.choice(specific_roads))
            ll_interm = ll_map.laneletLayer[interm]
            route = stitch_route(routing_graph, all_roads, ll_start, ll_interm, ll_end)

        else:
            route = None if routing_graph.getRoute(ll_start, ll_end) is None else routing_graph.getRoute(ll_start, ll_end).shortestPath()
            

        if route is None:
            continue

        path_seq = lanelet2.core.LaneletSequence(route)
        if prioritize_short_len:
            if sum(lanelet2.geometry.length2d(ll) for ll in path_seq) < shortest_route_len:
                selected_route = path_seq

        else:
            if random.choice([True, False]):
                selected_route = path_seq

        routes.append(route)
        count += 1

    return selected_route



# ==============================================================================
# simple-scenario interface
# ==============================================================================
def create_xosc(path, output_path, req_road_types):
    fname = f"{path[0].id}_to_{path[-1].id}"
    
    if req_road_types is not None:
        folder_path = f"/work/mgood/xosc-sumo-converter/scenario_creation/scenarios/planned_scenarios/{fname}"
    else:
        folder_path = f"/work/mgood/xosc-sumo-converter/scenario_creation/scenarios/random_scenarios/{fname}"

    os.mkdir(folder_path)
    fpath = f"{folder_path}/{fname}.xosc"
    fheader = "<?xml version='1.0' encoding='utf-8'?>\n"

    all_points = []
    for point in path:
        
        for node in point.centerline:
            all_points.append((node.x, node.y))

    root = ET.Element("OpenSCENARIO")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "OpenScenario.xsd")

    ET.SubElement(root, "FileHeader", description=fname, author="unsimple-scenario", revMajor="1", revMinor="0", date=f"{datetime.now().isoformat()}")
    ET.SubElement(root, "CatalogLocations")
    road_net = ET.SubElement(root, "RoadNetwork")
    ET.SubElement(road_net, "LogicFile", filepath="/work/mgood/xosc-sumo-converter/xodr/flat_campus.xodr")

    # this is probably where things could get less hard-coded (pass a vehicle dict along with its points?)
    entities = ET.SubElement(root, "Entities")
    scen_obj = ET.SubElement(entities, "ScenarioObject", name="ego_vehicle")
    veh = ET.SubElement(scen_obj, "Vehicle", name="car_white", vehicleCategory="car")
    bound_box = ET.SubElement(veh, "BoundingBox")
    ET.SubElement(bound_box, "Center", x="2.0", y="0.0", z="0.9")
    ET.SubElement(bound_box, "Dimensions", width="1.61", length="4.508", height="1.8")
    ET.SubElement(veh, "Performance", maxSpeed="50.8", maxDeceleration="11.5", maxAcceleration="11.5")
    axles = ET.SubElement(veh, "Axles")
    ET.SubElement(axles, "FrontAxle", maxSteering="0.523598775598", wheelDiameter="0.8", trackWidth="1.68", positionX="2.98", positionZ="0.4")
    ET.SubElement(axles, "RearAxle", maxSteering="0.523598775598", wheelDiameter="0.8", trackWidth="1.68", positionX="0.0", positionZ="0.4")
    properties = ET.SubElement(veh, "Properties")
    ET.SubElement(properties, "Property", name="model_id", value="ego")
    ET.SubElement(properties, "Property", name="type", value="ego_vehicle")
    ET.SubElement(properties, "File", filepath="/work/mgood/esmini-demo_Linux/esmini-demo/resources/models/car_blue.osgb")

    story_board = ET.SubElement(root, "Storyboard")
    init = ET.SubElement(story_board, "Init")
    actions = ET.SubElement(init, "Actions")
    private = ET.SubElement(actions, "Private", entityRef="ego_vehicle")
    private_action2 = ET.SubElement(private, "PrivateAction")
    tel_act = ET.SubElement(private_action2, "TeleportAction")
    pos = ET.SubElement(tel_act, "Position")
    ET.SubElement(pos, "WorldPosition", x=str(all_points[0][0]), y=str(all_points[0][1]), h="1.570796")

    priv_speed = ET.SubElement(private, "PrivateAction")
    long_act = ET.SubElement(priv_speed, "LongitudinalAction")
    speed = ET.SubElement(long_act, "SpeedAction")
    ET.SubElement(speed, "SpeedActionDynamics", dynamicsShape="step", value="0.0", dynamicsDimension="time")
    target_speed = ET.SubElement(speed, "SpeedActionTarget")
    ET.SubElement(target_speed, "AbsoluteTargetSpeed", value="15.0")

    story = ET.SubElement(story_board, "Story", name="MyStory")
    act = ET.SubElement(story, "Act", name="MyAct")
    man_group = ET.SubElement(act, "ManeuverGroup", name="MyManeuverGroup", maximumExecutionCount="1")

    actors = ET.SubElement(man_group, "Actors", selectTriggeringEntities="false")
    ET.SubElement(actors, "EntityRef", entityRef="ego_vehicle")
    maneuver = ET.SubElement(man_group, "Maneuver", name="Maneuver_ego_vehicle")
    event = ET.SubElement(maneuver, "Event", name="Event_ego_vehicle", priority="parallel", maximumExecutionCount="1")
    action = ET.SubElement(event, "Action", name="AssignRouteAction_ego_vehicle")
    private_action3 = ET.SubElement(action, "PrivateAction")
    rout_act = ET.SubElement(private_action3, "RoutingAction")
    follow_traj = ET.SubElement(rout_act, "FollowTrajectoryAction")
    ET.SubElement(follow_traj, "TrajectoryFollowingMode", followingMode="position")

    time_ref = ET.SubElement(follow_traj, "TimeReference")
    ET.SubElement(time_ref, "Timing", domainAbsoluteRelative="relative", offset="0.0", scale="1.0")
    
    traj = ET.SubElement(follow_traj, "Trajectory", name="Trajectory_ego_vehicle", closed="false")
    shape = ET.SubElement(traj, "Shape")
    polyline = ET.SubElement(shape, "Polyline")
    
    curr_time = 0.0
    prev_point = None

    for point in all_points[::5]:   # down sampling on points used to get smoother path and driving behavior
        if prev_point is not None:
            distance = np.sqrt((point[0] - prev_point[0])**2 + (point[1] - prev_point[1])**2)

            time_step = distance / 15.0
            curr_time += time_step

        else:
            curr_time = 0.0

        vertex = ET.SubElement(polyline, "Vertex", time=str(curr_time))
        pos1 = ET.SubElement(vertex, "Position")
        ET.SubElement(pos1, "WorldPosition", x=str(point[0]), y=str(point[1]))
        prev_point = point

    start_trig = ET.SubElement(event, "StartTrigger")
    start_cond_group = ET.SubElement(start_trig, "ConditionGroup")
    condition = ET.SubElement(start_cond_group, "Condition", name="InstantStartCondition", delay="0", conditionEdge="rising")
    by_val = ET.SubElement(condition, "ByValueCondition")
    ET.SubElement(by_val, "SimulationTimeCondition", value="0", rule="greaterThan")

    # act_start_trig = ET.SubElement(act, "StartTrigger")
    # act_cond_group = ET.SubElement(act_start_trig, "ConditionGroup")
    # act_condition = ET.SubElement(act_cond_group, "Condition", name="ActStartCondition", delay="0", conditionEdge="rising")
    # act_by_val = ET.SubElement(act_condition, "ByValueCondition")
    # ET.SubElement(act_by_val, "SimulationTimeCondition", value="0", rule="greaterThan")

    stop = ET.SubElement(story_board, "StopTrigger")
    cond_group = ET.SubElement(stop, "ConditionGroup")
    cond = ET.SubElement(cond_group, "Condition", name="Reach_End_Of_Path_Condition", delay="0", conditionEdge="rising")
    by_entity = ET.SubElement(cond, "ByEntityCondition")
    trig_entities = ET.SubElement(by_entity, "TriggeringEntities", selectTriggeringEntities="false")
    ET.SubElement(trig_entities, "EntityRef", entityRef="ego_vehicle")
    entity_cond = ET.SubElement(by_entity, "EntityCondition")
    reach_pos = ET.SubElement(entity_cond, "ReachPositionCondition", tolerance="2.0")
    pos_node = ET.SubElement(reach_pos, "Position")
    ET.SubElement(pos_node, "WorldPosition", x=str(all_points[-1][0]), y=str(all_points[-1][1]))

    with open(fpath, "w", encoding="utf-8") as f:
        f.write(fheader) 
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)  # Automatically handles formatting/tabs
        tree.write(f, encoding="unicode", xml_declaration=False)

    return folder_path, fpath


def run_scenario_with_esmini(xosc_path):
    esmini_path = "/work/mgood/xosc-sumo-converter/scenario_creation/bin/esmini"
    
    command = [
        esmini_path,
        "--window", "60", "60", "1024", "576",
        "--osc", xosc_path
    ]

    subprocess.run(command)



# ==============================================================================
# Miscellaneous functions
# ==============================================================================
def plot_route(all_roads, path, ll_map, folder_path, req_road_types):
    fig,ax = plt.subplots(figsize=(8,8))

    for road_id in all_roads:
        lanelet = ll_map.laneletLayer[int(road_id)]
        left = lanelet.leftBound
        right = lanelet.rightBound
        left_xy = [(pt.x, pt.y) for pt in left]
        right_xy = [(pt.x, pt.y) for pt in right]
        ax.plot(*zip(*left_xy), color='black', linewidth=2)
        ax.plot(*zip(*right_xy), color='black', linewidth=2)

    for node in path:
        node_coord = node.centerline[0]
        if roads_of_type is not None and str(node.id) in roads_of_type:
            ax.plot(node_coord.x, node_coord.y, marker="o", color="orange", markersize=2, label="_required_road_section")
        else:
            ax.plot(node_coord.x, node_coord.y, marker="o", color="green", markersize=2, label="_random_road_section") 
    
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', markersize=6, label='Required road section'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=6, label='Random road section')
    ]

    ax.set_aspect('equal')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"Ego Vehicle Route\nRoute Method: {', '.join(req_road_types) if req_road_types else 'random'}")
    ax.legend(handles=legend_elements, loc="upper left")
    ax.grid(True, alpha=0.3)
    
    try:
        plt.savefig(f"{folder_path}/plotted_route.png", dpi=400)
        print(f"Successfully plotted the route.")

    except Exception as e:
        print(f"Unable to plot the route. Error: {e}")



# ==============================================================================
# Main block
# ==============================================================================
if __name__ == "__main__":
    try:
        projector = LocalCartesianProjector(ORIGIN)
        ll2_map = SRP.load_map(OSM_FILE, projector)
        all_roads = find_all_roads(OSM_FILE, ll2_map, bounds=MAP_BOUNDS)
        req_road_types = None

        if req_road_types is not None:
            if type(req_road_types) != list:
                req_road_types = [req_road_types]
            
            roads_of_type = find_certain_roads(OSM_FILE, req_road_types)
        else:
            roads_of_type = None


        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        routing_graph = lanelet2.routing.RoutingGraph(ll2_map, traffic_rules)

        path = create_route(routing_graph, all_roads, ll2_map, specific_roads=roads_of_type, max_lookup=MAX_ROUTE_LOOKUP)
        folder_path, xosc_file_path = create_xosc(path, JSON_OUTPUT_PATH, req_road_types)
        plot_route(all_roads, path, ll2_map, folder_path, req_road_types)
        run_scenario_with_esmini(xosc_file_path)

    except KeyboardInterrupt:
        print(f"\nProgram interrupted by user")
    
    except Exception as e:
        print(f"The program was interrupted due to the following error: {e}")

    finally:
        delete_scenarios = input("\nWould you like to delete the created scenario information (y/n): ")

        if delete_scenarios.strip()[0].lower() == "y":
            
            folder_path = Path(folder_path)
            if folder_path.exists() and folder_path.is_dir():
                shutil.rmtree(folder_path)
                print(f"Successfully cleaned up directory: {folder_path.name}")

        else:
            print(f"Find relevant files at: {folder_path}")

        print("\nExiting program.")