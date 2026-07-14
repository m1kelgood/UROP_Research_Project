import traci
import xml.etree.ElementTree as ET
import os

#######################################################################
# Note: This has been developed to only handle a small set of scenarios
# that implement logic using world_position update and to create a full
# .rou.xml file from that. Additional development is required to support
# a wider range of cases
#######################################################################


def absolute_target_speed():
    '''
    The scenarios examined so far initialize the speed to 0, so this wouldn't
    be of any help (would need to calculate speed by distance and time information), which 
    would require improvising the parsing logic
    '''
    pass



def concat_route(veh_elem, edge_list):
    route = ""


    curr_edge = edge_list[0]
    next_edge = None
    final_edge = edge_list[-1]

    tot_route_dist = traci.simulation.getDistanceRoad(curr_edge, 0, final_edge, 0, isDriving=True)


    if len(edge_list) < 15:
        route_edges = traci.simulation.findRoute(curr_edge, final_edge)


    else:
        print("Edge list greater than 10", edge_list)
        # Need to develop this more before it is reliable

        # # arbitrary skip amount to prioritize accurate route following (some edges represent opposite lanes due to the OSC EdgeListforRoute method)
        # for edge in edge_list[3::3]: 
        #     next_edge = edge
        #     inter_route_dist = traci.simulation.getDistanceRoad(curr_edge, 0, next_edge, 0, isDriving=True)

        #     # for longer routes need a direction limit to prevent the vehicle from pursuing routes not on the intended path
        #     if inter_route_dist < tot_route_dist:
        #         edge_list = traci.simulation.findRoute(curr_edge, next_edge)
        #         route = route_list_update(route, edge_list)

        #         curr_edge = edge
            
        # if curr_edge != final_edge:
        #     edge_list = traci.simulation.findRoute(curr_edge, final_edge)
        #     route = route_list_update(route, edge_list)



    for edge in route_edges.edges:
        route += str(edge) + " "

    out_route = ET.SubElement(veh_elem, "route")
    out_route.set("edges", route)




def create_full_rou(route_commands, base_name, fheader, sumocfg):
    if sumocfg not in os.listdir():
        print(f"Couldn't find SUMO configuration file '{sumocfg}', exiting the program")
        quit()

    # Simulation set-up commands
    sumo_cmd = [
        "sumo", "-c", sumocfg, "--no-step-log", "true", "--no-warnings", "true"
    ]

    # Initialize the connection
    traci.start(sumo_cmd)  
    
    defaults = {
        "departLane": "best", "depart": "0.0", "departSpeed": "avg"
    }

    fname = base_name + "_FULL.rou.xml"

    print(f"\nCreating full event sequence (no dynamic update) to {os.getcwd()}/{fname}")

    root = ET.Element("routes")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/routes_file.xsd")

    for obj, commands in route_commands.items():
        
        if obj == "Ego":
            veh_elem = ET.SubElement(root, "vehicle", id=obj, color="255,255,255")
        else:
            veh_elem = ET.SubElement(root, "vehicle", id=obj)

        

        for command in commands:
            edges = command.get("EdgeListforRoute")
            if edges is not None:
                concat_route(veh_elem, edges)

    traci.close()

    for elem in root:
        for key, val in defaults.items():
            if key not in list(elem.attrib.keys()):
                print(f"{elem.attrib.get('id')} was missing necessary spawn attribtue: {key}")
                print(f"Default value of {val} being applied\n")
                elem.set(key, val)


    with open(fname, "w", encoding="utf-8") as f:
        f.write(fheader) 
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)  # Automatically handles formatting/tabs
        tree.write(f, encoding="unicode", xml_declaration=False)


    return fname



def create_other_sumocfg(rou_file, net_file, base_name, fheader):
    fname = base_name + "_FULL.sumocfg"

    
    root = ET.Element("sumoConfiguration")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/sumoConfiguration.xsd")
    
    input_elem = ET.SubElement(root, "input")
    ET.SubElement(input_elem, "net-file", value="../../" + net_file)
    ET.SubElement(input_elem, "route-files", value=rou_file)

    time_elem = ET.SubElement(root, "time")
    ET.SubElement(time_elem, "step-length", value="0.1")

    proc_elem = ET.SubElement(root, "processing")
    ET.SubElement(proc_elem, "lateral-resolution", value="0.1")
    ET.SubElement(proc_elem, "time-to-teleport", value="-1")


    with open(fname, "w", encoding="utf-8") as f:
        f.write(fheader) 
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)  # Automatically handles formatting/tabs
        tree.write(f, encoding="unicode", xml_declaration=False)