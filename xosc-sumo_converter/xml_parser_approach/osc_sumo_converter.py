import os
import xml.etree.ElementTree as ET
import sumolib



# ==============================================================================
# Action Handler Functions
# ==============================================================================
def lane_position(veh_elem, attributes):
    for key, val in attributes.items():
        if key == "roadId":
            route = ET.SubElement(veh_elem, "route")
            route.set("edges", val)
        elif key == "laneId":
            veh_elem.set("departLane", val)
        elif key == "s":
            veh_elem.set("departPos", val)


def speed_action_dynamics(veh_elem, attributes):
    # So far every case has been a step up to full speed, so no other cases are being handled
    veh_elem.set("depart", "0.0")


def absolute_target_speed(veh_elem, attributes):
    for key, val in attributes.items():
        veh_elem.set("departSpeed", val)


def route_info(veh_elem, attributes):
    ''' 
    The edges provided in the route file aren't connected, so pass the first edge so that the .rou file
    can be created, and then save the rest of the route construction for traci.
    '''
    edges = ""
    count = 0
    for road, _ in attributes:
        if count > 0:
            break
        edges += road + " "
        count += 1

    route = ET.SubElement(veh_elem, "route")
    route.set("edges", edges)

    

# add to this as necessary (when experimenting with new files)
ACTION_MAPPING = {
    "LanePosition": lane_position,
    "SpeedActionDynamics": speed_action_dynamics,
    "AbsoluteTargetSpeed": absolute_target_speed,
    "RouteInfo": route_info,
}



# ==============================================================================
# read_text() helper functions
# ==============================================================================
def find_value(ref_name, param_commands):
    '''
    Find a value when the text is referencing parameter declaration. The structure
    of this function was modified to handle the cases introduced in 'synchronize.xosc',
    for values references of the form '${$EgoStartS + 50}'
    '''
    return_val = ref_name

    count = 0
    while return_val.startswith("$"):
        sign_count = return_val.count("$")
        print(return_val, sign_count, param_commands, '\n')

        if sign_count == 1:

            if return_val.startswith("${"):
                operation = return_val[1:].strip("}{").split()[0]
                new_val = eval(operation)
                return_val = str(new_val)

            elif return_val.startswith("$"):
                if return_val in list(param_commands.keys()):
                    return_val = param_commands[return_val]

                elif return_val[1:] in list(param_commands.keys()):
                    return_val = param_commands[return_val[1:]]
            
        else:
            sequence = return_val[1:].strip("}{").split()

            init_val = param_commands[sequence[0][1:]]
            update = "".join(sequence[1:])
            operation = init_val + update
            
            new_val = eval(operation)
            return_val = str(new_val)

        count += 1
        if count > 10:
            print("Error with a parameter reference value, exiting the program.")
            quit()


    return return_val


def parameters(child, param_commands):
    for obj in child:
        
        if child.tag == "ParameterAssignments":
            name = obj.attrib.get("parameterRef")
        else:
            name = obj.attrib.get("name")

        value = obj.attrib.get("value")

        if name != value:
            param_commands[name] = value

    return param_commands


def catalog(child, catalog_commands):

    for catalog_type in child:
        for catalogs in catalog_type:
            path = catalogs.attrib.get("path")

        catalog_commands[catalog_type.tag] = {"path": path}

    return catalog_commands


def entity(child, spawn_commands, catalog_commands):

    for obj in child:
        name = obj.attrib["name"]

        for descriptions in obj:
            if descriptions.attrib:
                catalog_ref = descriptions.attrib.get("catalogName")
                entry_name = descriptions.attrib.get("entryName")

                if catalog_ref in catalog_commands.keys():
                    catalog_commands[catalog_ref][name] = entry_name

            spawn_commands[name] = []

    return spawn_commands, catalog_commands


def init(child, spawn_commands, param_commands):
    print("\nINIT START")
    curr_obj = None

    for command in child.iter():
        if command.tag == "Private":
            curr_obj = command.attrib.get("entityRef")

            if curr_obj not in spawn_commands:
                spawn_commands[curr_obj] = []

            continue

        for descr in command:
            new_attrib = {}
            for key, val in descr.items():
                if val.startswith("$"):
                    cat_val = find_value(val, param_commands)
                    new_attrib[key] = cat_val
                else:
                    new_attrib[key] = val

            if curr_obj != None and new_attrib:
                spawn_commands[curr_obj].append({descr.tag: new_attrib})

    print("INIT END\n")
    return spawn_commands


def story(child, move_commands, param_commands):
    print("\nSTORY START")
    curr_obj = None

    for command in child.iter():
        if command.tag == "ParameterDeclarations":
            param_commands = parameters(command, param_commands)

        elif command.tag == "EntityRef":
            curr_obj = command.attrib.get("entityRef")

            if curr_obj.startswith("$"):
                curr_obj = find_value(curr_obj, param_commands)

            if curr_obj not in move_commands:
                move_commands[curr_obj] = []

            continue


        for descr in command:
            new_attrib = {}
            for key, val in descr.items():
                if val.startswith("$"):
                    cat_val = find_value(val, param_commands)
                    new_attrib[key] = cat_val

                else:
                    new_attrib[key] = val

            if curr_obj != None and new_attrib:
                move_commands[curr_obj].append({descr.tag: new_attrib})

    print("STORY END\n")

    return move_commands



# ==============================================================================
# process_text() helper functions
# ==============================================================================
def lane_edge_mapping(net_file_path):
    '''
    OpenDrive/OpenScenario has a different road & lane numbering system than SUMO,
    and this error occurs during netconvert. By using the metadata in the .net.xml
    file that was generated via netconvert, this mapping table can solve the issue.
    '''
    tree = ET.parse(net_file_path)
    root = tree.getroot()

    mapping_table = {}
    
    for edge in root.findall('edge'):

        new_edge_id = edge.attrib.get("id")

        for lane in edge.findall('lane'):

            new_lane_idx = lane.attrib.get("index")
            lane_len = lane.attrib.get("length")

            param_elem = lane.find("./param[@key='origId']")

            if param_elem is not None:
                orig_edge_id, orig_lane_id = param_elem.attrib.get("value").split("_")
                key = (orig_edge_id, orig_lane_id, lane_len)

                
                mapping_table[key] = (new_edge_id, new_lane_idx)
    
    return mapping_table


def update_lane_edge(command_dict, mapping_table):
    '''
    Update the roadId and laneId according to the mapping table from the 
    lane_edge_mapping() function.
    '''
    for obj, commands in command_dict.items():

        for index, command in enumerate(commands):
            if "LanePosition" in list(command.keys()):
                curr_road = command.get("LanePosition").get("roadId")
                curr_lane = command.get("LanePosition").get("laneId")
                depart_offset = command.get("LanePosition").get("s")

                sumo_edge, sumo_lane_idx = None, None

                for key, value in mapping_table.items():
                    if key[0] == curr_road and key[1] == curr_lane:

                        if sumo_edge is None and sumo_lane_idx is None:
                            sumo_edge = value[0]
                            sumo_lane_idx = value[1]

                        else:
                            if depart_offset > key[2]:
                                sumo_edge = value[0]
                                sumo_lane_idx = value[1]
                
                command_dict[obj][index]["LanePosition"]["roadId"] = sumo_edge
                command_dict[obj][index]["LanePosition"]["laneId"] = sumo_lane_idx

    return command_dict


def catalog_reference(spawn_dict, move_dict, catalog_data, mapping_table):

    for obj, commands in spawn_dict.items():

        for index, command in enumerate(commands):
            if "CatalogReference" in list(command.keys()):
                print(index, command)
                catalog_name = command.get("CatalogReference").get("catalogName")
                catalog_entry = command.get("CatalogReference").get("entryName")

                for catalog in catalog_data:
                    path = catalog_data[catalog].get("path")

                    if catalog_name + ".xosc" in os.listdir(path):
                        full_file = path + "/" + catalog_name + ".xosc"
                        spawn_dict, move_dict = resolve_reference(obj, spawn_dict, move_dict, mapping_table, full_file, catalog_entry)
                        spawn_dict[obj].remove(command)

    return spawn_dict, move_dict


def resolve_reference(obj, spawn_dict, move_dict, mapping_table, full_file, catalog_entry):

    tree = ET.parse(full_file)
    root = tree.getroot()

    for child in root:
        for grandchild in child:
            name = grandchild.attrib.get("name")

            if name == catalog_entry:
                route = []

                for detail in grandchild.iter():

                    if detail.tag == "LanePosition":
                        road_id = detail.attrib.get("roadId")
                        lane_id = detail.attrib.get("laneId")

                        for key, value in mapping_table.items():
                            if key[0] == road_id and key[1] == lane_id:
                                route.append(value)
                                continue

                if {"RouteInfo": route} not in move_dict[obj]:
                    move_dict[obj].append({"RouteInfo": route})
                    spawn_dict[obj].append({"RouteInfo": route})

    return spawn_dict, move_dict


def edges_from_point_route(move_data, net_file_path):

    for obj, command_list in move_data.items():
        edge_route = []
        filtered_commands = []

        for command in command_list:
            if list(command.keys())[0] == "Vertex":
                continue
            
            elif list(command.keys())[0] == "WorldPosition":
                attributes = next(iter(command.values()))
                edge = edge_from_coord(None, attributes, net_file_path)
                
                if len(edge_route) == 0:
                    edge_route.append(edge)
                else:
                    if edge != edge_route[-1]:
                        edge_route.append(edge)

            else:
                filtered_commands.append(command)

        if edge_route:
            filtered_commands.append({"EdgeListforRoute": edge_route})
        
        move_data[obj] = filtered_commands

    return move_data


# ==============================================================================
# output_text() helper functions
# ==============================================================================
def rel_road_pos(veh_elem, attributes, spawn_commands):
    ref_veh = attributes.get("entityRef")
    s = float(attributes.get("ds"))
    lane = float(attributes.get("dt")) // 3.1

    if ref_veh in list(spawn_commands.keys()):

        lp_ind = None
        for index, command in enumerate(spawn_commands[ref_veh]):
            if "LanePosition" in command:
                lp_ind = index

        if lp_ind is not None:
            info = spawn_commands[ref_veh][lp_ind]["LanePosition"]
            road = info.get("roadId")

            for key, val in info.items():
                if key == "laneId":
                    lane += int(val)
                elif key == "s":
                    s += float(val)

            veh_elem.set("departLane", str(int(lane)))
            veh_elem.set("departPos", str(s))
            
            route = ET.SubElement(veh_elem, "route")
            route.set("edges", road)

            
def edge_from_coord(veh_elem, attributes, net_file):
    x = float(attributes.get("x"))
    y = float(attributes.get("y"))

    try:
        net = sumolib.net.readNet(net_file)
    except FileNotFoundError:
        net = sumolib.net.readNet("../../" + net_file)

    x_off, y_off = net.getLocationOffset()
    x_sumo = x + x_off
    y_sumo = y + y_off
    radius = 1
    selected_edge = None

    while selected_edge is None:
        edges = net.getNeighboringEdges(x_sumo, y_sumo, radius)

        if len(edges) > 0:
            distances_and_edges = sorted([(dist, edge) for edge, dist in edges], key=lambda x:x[0])
            selected_edge = distances_and_edges[0][-1]
        else:
            radius *= 2

    # Allowing this function to be compatible with creating the rou file and returning an edge
    if veh_elem is not None:
        route = ET.SubElement(veh_elem, "route")
        route.set("edges", selected_edge._id)
    
    else:
        return selected_edge._id


def create_rou(spawn_commands, base_name, fheader, net_file):
    defaults = {
        "depart": "0.0", "departSpeed": "5.0"
    }
    additional_dynamic_commands = {}
    fname = base_name + ".rou.xml"

    root = ET.Element("routes")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/routes_file.xsd")


    for obj, commands in spawn_commands.items():
        # print(obj)
        
        if obj == "Ego":
            veh_elem = ET.SubElement(root, "vehicle", id=obj, color="255,255,255")
        else:
            veh_elem = ET.SubElement(root, "vehicle", id=obj)

        

        for command in commands:
            for decl, attributes in command.items():
                # print("\t"*1, decl)
                # print("\t"*2, attributes)
                if decl in ACTION_MAPPING:
                    # verify that the following commented out code works as intended (will need to modify ACTION_MAPPING functions)
                    ACTION_MAPPING[decl](veh_elem, attributes)
                elif decl == "RelativeTargetSpeed":
                    additional_dynamic_commands[obj] = [{decl: attributes}]
                elif decl == "RelativeRoadPosition":
                    rel_road_pos(veh_elem, attributes, spawn_commands)
                elif decl == "WorldPosition":
                    edge_from_coord(veh_elem, attributes, net_file)

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

    
    return fname, additional_dynamic_commands


def create_sumocfg(rou_file, net_file, base_name, fheader):
    fname = base_name + ".sumocfg"
    
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

    return fname



# ==============================================================================
# Primary pipeline functions
# ==============================================================================
def read_text(input_path):
    print(f"\nReading OpenSCENARIO from: {input_path}")
    
    spawn_commands, move_commands, catalog_commands, param_commands = {}, {}, {}, {}

    tree = ET.parse(input_path)
    root = tree.getroot()

    for child in root:
        if child.tag == "ParameterDeclarations":
            param_commands = parameters(child, param_commands)
        
        elif child.tag == "CatalogLocations":
            catalog_commands = catalog(child, catalog_commands)
        
        elif child.tag == "Entities":
            spawn_commands, catalog_commands = entity(child, spawn_commands, catalog_commands)

        elif child.tag == "Storyboard":
            for grandchild in child:
                if grandchild.tag == "Init":
                    spawn_commands = init(grandchild, spawn_commands, param_commands)
                elif grandchild.tag == "Story":
                    move_commands = story(grandchild, move_commands, param_commands)
        
    return spawn_commands, move_commands, catalog_commands


def process_text(spawn_data, move_data, catalog_data, net_file_path):
    print(f"\nProcessing data using network: {net_file_path}")
    
    mapping_table = lane_edge_mapping(net_file_path)

    spawn_data = update_lane_edge(spawn_data, mapping_table)
    move_data = update_lane_edge(move_data, mapping_table)

    spawn_data, move_data = catalog_reference(spawn_data, move_data, catalog_data, mapping_table)
    move_data = edges_from_point_route(move_data, net_file_path)

    return spawn_data, move_data, mapping_table


def output_text(sumo_spawn, output_dir, net_file, base_name, fheader):
    print(f"\nWriting SUMO files to directory: {output_dir}\n")
    
    try:
        os.chdir(output_dir)
    except Exception:
        os.mkdir(output_dir)
        os.chdir(output_dir)

    rou_file, additional_dynamic_commands = create_rou(sumo_spawn, base_name, fheader, net_file)
    sumocfg_file = create_sumocfg(rou_file, net_file, base_name, fheader)

    return sumocfg_file, additional_dynamic_commands