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
import shutil
from datetime import datetime
import numpy as np
import argparse
import json
from scenario_setup import create_header, create_entities, create_init, create_story



# ==============================================================================
# Global variables
# ==============================================================================
DEFAULT_OSM_FILE = Path("../../osm/campus.osm")

# this needs to match the geoReference line in the .xodr file
DEFUALT_ORIGIN = Origin(50.785562924295, 6.04648796549892, 0)

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
def create_xosc(path, input_info, req_road_types):
    all_points = []
    for point in path:
        
        for node in point.centerline:
            all_points.append((node.x, node.y))
    
    with open(input_info, "r") as f:
        info = json.load(f)
        
    fname = info.get("scenario name")
    vehicle_dict = info.get("vehicles")
    
    if req_road_types is not None:
        folder_path = f"../scenarios/planned_scenarios/{fname}"
    else:
        folder_path = f"../scenarios/random_scenarios/{fname}"

    os.mkdir(folder_path)
    fpath = f"{folder_path}/{fname}.xosc"
    fheader = "<?xml version='1.0' encoding='utf-8'?>\n"

    root = create_header(info.get("scenario name"), info.get("vehicle catalog"), info.get("xodr file"))
    root = create_entities(root, vehicle_dict)
    
    story_board, vehicle_dict = create_init(root, vehicle_dict, all_points)
    try:
        story_board = create_story(story_board, info.get("story name"), vehicle_dict, all_points)
    except Exception:
        traceback.print_exc()

    with open(fpath, "w", encoding="utf-8") as f:
        f.write(fheader) 
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(f, encoding="unicode", xml_declaration=False)


    # start_trig = ET.SubElement(event, "StartTrigger")
    # start_cond_group = ET.SubElement(start_trig, "ConditionGroup")
    # condition = ET.SubElement(start_cond_group, "Condition", name="InstantStartCondition", delay="0", conditionEdge="rising")
    # by_val = ET.SubElement(condition, "ByValueCondition")
    # ET.SubElement(by_val, "SimulationTimeCondition", value="0", rule="greaterThan")

    # act_start_trig = ET.SubElement(act, "StartTrigger")
    # act_cond_group = ET.SubElement(act_start_trig, "ConditionGroup")
    # act_condition = ET.SubElement(act_cond_group, "Condition", name="ActStartCondition", delay="0", conditionEdge="rising")
    # act_by_val = ET.SubElement(act_condition, "ByValueCondition")
    # ET.SubElement(act_by_val, "SimulationTimeCondition", value="0", rule="greaterThan")

    # stop = ET.SubElement(story_board, "StopTrigger")
    # cond_group = ET.SubElement(stop, "ConditionGroup")
    # cond = ET.SubElement(cond_group, "Condition", name="Reach_End_Of_Path_Condition", delay="0", conditionEdge="rising")
    # by_entity = ET.SubElement(cond, "ByEntityCondition")
    # trig_entities = ET.SubElement(by_entity, "TriggeringEntities", selectTriggeringEntities="false")
    # ET.SubElement(trig_entities, "EntityRef", entityRef="ego_vehicle")
    # entity_cond = ET.SubElement(by_entity, "EntityCondition")
    # reach_pos = ET.SubElement(entity_cond, "ReachPositionCondition", tolerance="2.0")
    # pos_node = ET.SubElement(reach_pos, "Position")
    # ET.SubElement(pos_node, "WorldPosition", x=str(all_points[-1][0]), y=str(all_points[-1][1]))

    return folder_path, fpath



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
        parser = argparse.ArgumentParser(description='Create an OpenSCENARIO driving scene given an input file')
        parser.add_argument("--osm", type=Path, default=DEFAULT_OSM_FILE, help="OSM file from which the route will be derived")
        parser.add_argument("--origin", type=tuple, default=(50.785562924295, 6.04648796549892))
        parser.add_argument("--input_info", type=Path, default=Path("default_input_info.json"), help="information from which the scenario will be created")
        args = parser.parse_args()
        
        if len(args.origin) >= 2:
            origin = Origin(args.origin[0], args.origin[1], 0)
        else:
            origin = DEFUALT_ORIGIN


        projector = LocalCartesianProjector(origin)
        ll2_map = SRP.load_map(str(args.osm), projector)
        all_roads = find_all_roads(str(args.osm), ll2_map, bounds=MAP_BOUNDS)
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
        folder_path, xosc_file_path = create_xosc(path, args.input_info, req_road_types)
        plot_route(all_roads, path, ll2_map, folder_path, req_road_types)

    except KeyboardInterrupt:
        print(f"\nProgram interrupted by user")
    
    except Exception as e:
        print(f"The program was interrupted due to the following error: {e}")

    finally:
        quit()
        delete_scenarios = input("\nWould you like to delete the created scenario information (y/n): ")

        if delete_scenarios.strip()[0].lower() == "y":
            
            folder_path = Path(folder_path)
            if folder_path.exists() and folder_path.is_dir():
                shutil.rmtree(folder_path)
                print(f"Successfully cleaned up directory: {folder_path.name}")

        else:
            print(f"Find relevant files at: {folder_path}")

        print("\nExiting program.")