import traci
import os
import math
import time



# ==============================================================================
# Action handler functions
# ==============================================================================
def control_relative_speed(obj, output):
    current_speed = traci.vehicle.getSpeed(obj)
    desired_speed = 0
    target_vehicle = output.get("entityRef")

    if target_vehicle in traci.vehicle.getIDList():
        desired_speed += traci.vehicle.getSpeed(target_vehicle)
        update_value = output.get("value")
        update_type = output.get("type")

        if update_type == "delta":
            desired_speed += update_value

        elif update_type == "factor":
            desired_speed *= update_value

        # For some unknown reason, setSpeed was not working, so setPreviousSpeed is being used instead
        traci.vehicle.setPreviousSpeed(obj, desired_speed)

        if abs(desired_speed - current_speed) > 1:
            print(f"Successfully updated {obj}'s speed from {current_speed} to {desired_speed} (ref. vehicle: {target_vehicle}).")

    else:
        print(f"Vehicle {target_vehicle} is not in the simulation. {obj}'s speed update failed.")


ACTION_MAPPING = {
    "RelativeTargetSpeed": control_relative_speed,
}



# ==============================================================================
# Other general functions
# ==============================================================================
def reroute(vehicle):
    possible_routes = []
    current_edge = traci.vehicle.getRoadID(vehicle)
    junctions = traci.junction.getIDList()


    for junction in junctions:
        valid_edges = [edge for edge in traci.junction.getIncomingEdges(junction) if not edge.startswith(":")]

        if valid_edges:
            vehicle_position = traci.vehicle.getPosition(vehicle)
            junction_position = traci.junction.getPosition(junction)
            
            dx = junction_position[0] - vehicle_position[0]
            dy = junction_position[1] - vehicle_position[1]
            route_distance = math.sqrt(dx**2 + dy**2)

            possible_routes.append([route_distance, valid_edges])


    possible_routes.sort(key=lambda d: next(iter(d)), reverse=True)
    for route in possible_routes:
        for potential_edge in route[-1]:
            new_route = traci.simulation.findRoute(current_edge, potential_edge)
            
            # If a new route has been chosen and is valid, the edge list won't be empty
            if new_route.edges:
                traci.vehicle.changeTarget(vehicle, potential_edge)
                break
        
        else:
            continue

        break


def lane_change(obj, output, target_lane, mapping_table):

    lane_position = traci.vehicle.getLanePosition(obj)
    current_lane = traci.vehicle.getLaneIndex(obj)
    current_edge = traci.vehicle.getRoadID(obj)

    desired_lane = None
    desired_edge = output.get("roadId")
    desired_position = float(output.get("s"))

    if current_edge == desired_edge and lane_position >= desired_position:
        for key, value in mapping_table.items():
            if key[1] == target_lane and value[0] == desired_edge:
                desired_lane = value[1]

        if desired_lane is not None:
            print(f"Lane change underway for {obj}: curr_pos={lane_position:.3f}, curr_lane={current_lane}, current_edge={current_edge}, desired_position={desired_position}, desired_lane={desired_lane}, desired_edge={desired_edge}")
            traci.vehicle.changeLane(obj, desired_lane, duration=100)

        else:
            print(f"Error with desired lane. {obj} couldn't complete the lane change.\n")


def relative_lane_change(obj, output, command):
    ref_veh = output.get("entityRef")
    lane_offset = output.get("value")

    curr_edge = traci.vehicle.getRoadID(obj)
    num_lanes_on_edge = traci.edge.getLaneNumber(curr_edge)

    obj_lane_id = traci.vehicle.getLaneID(obj)
    obj_road_id = traci.vehicle.getRoadID(obj)
    ref_veh_lane_id = traci.vehicle.getLaneID(ref_veh)
    ref_veh_road_id = traci.vehicle.getRoadID(ref_veh)
    
    if num_lanes_on_edge > 1 and obj_road_id == ref_veh_road_id:

        obj_pos = traci.vehicle.getLanePosition(obj)
        ref_veh_pos = traci.vehicle.getLanePosition(ref_veh)
        ref_veh_len = traci.vehicle.getLength(ref_veh)

        if obj_lane_id == ref_veh_lane_id:
            new_lane_id = obj_lane_id[:-1] + lane_offset
            traci.vehicle.moveTo(obj, new_lane_id, obj_pos)
            print(f"{obj} completed a lane change from {obj_lane_id} to {new_lane_id}.")
            return True

        elif obj_pos > (ref_veh_pos + ref_veh_len) * 1.1:
            new_lane_id = obj_lane_id[:-1] + lane_offset
            traci.vehicle.moveTo(obj, ref_veh_lane_id, obj_pos)
            print(f"{obj} completed a lane change from {obj_lane_id} to {new_lane_id}.")
            return True

    return False


def update_speed(obj, output, speed_change_details):
    step_len = traci.simulation.getDeltaT()

    current_speed = traci.vehicle.getSpeed(obj)
    desired_speed = float(output.get("value"))
    duration = float(speed_change_details.get("value"))

    if duration != 0:
        speed_delta = ((desired_speed - current_speed)*step_len)/(duration)

    else:
        speed_delta = desired_speed - current_speed

    print(f"UPDATE SPEED {current_speed, desired_speed, speed_delta}")
    if obj in traci.vehicle.getIDList():
        
        if abs(traci.vehicle.getSpeed(obj) - desired_speed) > 1:
            traci.vehicle.setPreviousSpeed(obj, current_speed + speed_delta)
            print(f"Successfully updated {obj}'s speed from {current_speed} to {traci.vehicle.getSpeed(obj)}.")

        else:
            print(f"{obj}'s speed is already at the desired speed ({current_speed}).")

    else:
        print(f"{obj}'s speed update failed.")
    


def command_update(command, veh_list, mapping_table):

    for obj, instructions in command.items():
        
        if obj in veh_list:
            target_lane = None
            speed_change_details = None

            for instruction in instructions:
                for title, output in instruction.items():
                    print("\t", title, output)

                    if title == "AbsoluteTargetLane":
                        target_lane = output.get("value")

                    elif title == "SpeedActionDynamics":
                        speed_change_details = output
                    
                    elif title == "LanePosition" and target_lane is not None:
                        lane_change(obj, output, target_lane, mapping_table)

                    elif title == "RelativeTargetLane":
                        complete = relative_lane_change(obj, output, command)
                        
                        if complete:
                            command[obj].remove({title:output})
                            print(f"The event '{title}' has been completed")

                    elif title == "AbsoluteTargetSpeed":
                        update_speed(obj, output, speed_change_details)

                    elif title in ACTION_MAPPING:
                        ACTION_MAPPING[title](obj, output)

    return command


def route_list_update(route_list, traci_route):
    '''
    Small helper function to avoid repetitive code in the check_for_route function
    '''
    for edge in traci_route.edges:
        if len(route_list) == 0:
            route_list.append(edge)
        else:
            if edge != route_list[-1]:
                route_list.append(edge)

    return route_list



def check_for_route(vehicle, command_dict):
    route = []

    if vehicle not in list(command_dict.keys()):
        return route
    
    for commands in command_dict[vehicle]:
        route_info = commands.get("RouteInfo")
        edges_for_route = commands.get("EdgeListforRoute")

        
        if route_info is not None:
            for i in range(len(route_info)):
                if i < len(route_info)-1:
                    edge_i = route_info[i][0]
                    edge_f = route_info[i+1][0]
    
                    new_route = traci.simulation.findRoute(edge_i, edge_f)
                    route = route_list_update(route, new_route)

        elif edges_for_route is not None:

            tot_route_dist = traci.simulation.getDistanceRoad(edges_for_route[0], 0, edges_for_route[-1], 0, isDriving=True)

            curr_edge = edges_for_route[0]
            next_edge = None
            final_edge = edges_for_route[-1]

            # arbitrary skip amount to prioritize accurate route following (some edges represent opposite lanes due to the OSC EdgeListforRoute method)
            for edge in edges_for_route[3::3]: 
                next_edge = edge
                inter_route_dist = traci.simulation.getDistanceRoad(curr_edge, 0, next_edge, 0, isDriving=True)

                # for longer routes need a direction limit to prevent the vehicle from pursuing routes not on the intended path
                if inter_route_dist < tot_route_dist:
                    edge_list = traci.simulation.findRoute(curr_edge, next_edge)
                    route = route_list_update(route, edge_list)

                    curr_edge = edge
                
            if curr_edge != final_edge:
                edge_list = traci.simulation.findRoute(curr_edge, final_edge)
                route = route_list_update(route, edge_list)


            
            # if len(edges_for_route) >= 2 and len(edges_for_route) <= 10:
            #     edge_list = traci.simulation.findRoute(edges_for_route[0], edges_for_route[-1]).edges
            #     route = list(edge_list)

            # else:
            #     print(edges_for_route)
            #     tot_route_dist = traci.simulation.getDistanceRoad(edges_for_route[0], 0, edges_for_route[-1], 0, isDriving=True)

            #     print(f"TRD {tot_route_dist}")

            #     curr_edge = edges_for_route[0]
            #     next_edge = None
            #     final_edge = edges_for_route[-1]

            #     for edge in edges_for_route[3::3]:
            #         next_edge = edge

            #         print(f"CURR, NEXT {curr_edge, next_edge}")
            #         inter_route_dist = traci.simulation.getDistanceRoad(curr_edge, 0, next_edge, 0, isDriving=True)
            #         print(f"IRD {inter_route_dist}")

            #         if inter_route_dist < tot_route_dist:
            #             edge_list = traci.simulation.findRoute(curr_edge, next_edge)
            #             route = route_list_update(route, edge_list)
    
            #             curr_edge = edge
                    
            #     print(curr_edge, final_edge)
            #     if curr_edge != final_edge:
            #         edge_list = traci.simulation.findRoute(curr_edge, final_edge)
            #         print(edge_list.edges)
            #         route = route_list_update(route, edge_list)
                    
            #     print(route)

    return route



# ==============================================================================
# Simulation Control and Main Block
# ==============================================================================
def traci_control(additional_dynamic_commands, traci_move, mapping_table, output_dir, sumocfg, startup):
    # Ensure .sumocfg file exists in the current directory
    if sumocfg not in os.listdir():
        print(f"Couldn't find SUMO configuration file '{sumocfg}', exiting the program")
        quit()

    # Simulation set-up commands
    sumo_cmd = [
        startup.get("interface"), "-c", sumocfg, "--start", "--quit-on-end", "--step-length", startup.get("step-length"),
        "--delay", startup.get("delay"), "--waiting-time-memory", startup.get("waiting-time-memory"), "--lanechange-output",
        "lanechange_data.xml"
    ]

    # Initialize the connection
    traci.start(sumo_cmd)  

    if startup.get("interface") == "sumo-gui":
        print(f"\n––– Launching the Traci-Controlled SUMO Simulation (GUI version) using {sumocfg} –––\n")
        # sumo_cmd.extend(["--gui-settings-file", "view_settings.xml"])
        traci.gui.setSchema("View #0", "real world")

    else:
        print(f"\n––– Launching the Traci-Controlled SUMO Simulation (non-GUI version) using {sumocfg} –––\n")
      

    try:

        step = 0
        rerouted_vehicles = []
        time_quit = float(startup.get("desired_sim_duration")) / float(startup.get("step-length"))

        # Begin the simulation loop
        while traci.simulation.getMinExpectedNumber() > 0 and step < time_quit:
            # Ensure all the vehicles spawn before making any commands
            traci.simulationStep()
            step += 1

            current_vehicles = traci.vehicle.getIDList()
            print(f"Vehicles currently in the simulation at step {step}: {current_vehicles}.")

            if len(rerouted_vehicles) != len(current_vehicles):
                for vehicle in current_vehicles:
                    route = check_for_route(vehicle, traci_move)

                    if vehicle not in rerouted_vehicles:

                        if route:
                            traci.vehicle.setRoute(vehicle, route)
                            print(f"VEH ROUTE {traci.vehicle.getRoute(vehicle)}")

                        # rerouting vehicles if no other route is defined so they don't despawn so quickly
                        else:
                            reroute(vehicle)

                        rerouted_vehicles.append(vehicle)
                        # depending on the desired traffic dynamics, adjust the LaneChangeMode
                        # traci.vehicle.setLaneChangeMode(vehicle, 0b001000000000)
            
            if step == 1:
                time.sleep(1)  # for debugging
            
            print("—"*80)
            print("DYNAMIC UPDATES")
            print("—"*80)
            additional_dynamic_commands = command_update(additional_dynamic_commands, current_vehicles, mapping_table)
            traci_move = command_update(traci_move, current_vehicles, mapping_table)
            print("\n")


    except KeyboardInterrupt:
        print("\n>>> Simulation interrupted by user")
    
    finally:
        # Close the connection
        traci.close()

    print(additional_dynamic_commands, "\n")
    print(traci_move)