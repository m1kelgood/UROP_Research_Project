import xml.etree.ElementTree as ET
from datetime import datetime
import random


def create_header(description, veh_cat_path, xodr_file):
    root = ET.Element(
        "OpenSCENARIO",
        {
            "xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance", "xsi:noNamespaceSchemaLocation": "OpenScenario.xsd",
        }
    )


    file_header = ET.SubElement(root, "FileHeader",
        {
            "description": description,
            "author": "unsimple-scenario",
            "revMajor": "1",
            "revMinor": "0",
            "date": str(datetime.now())
        }
    )


    catalog_locs = ET.SubElement(root, "CatalogLocations")
    veh_cat = ET.SubElement(catalog_locs, "Directory",
        {
            "path": veh_cat_path
        }
    )


    road_net = ET.SubElement(root, "RoadNetwork")
    logic_file = ET.SubElement(road_net, "LogicFile",
        {
            "filepath": xodr_file
        }
    )


    return root



def create_entities(root, veh_dict):
    entities = ET.SubElement(root, "Entities")
    
    for veh, attrib in veh_dict.items():
        scen_obj = ET.SubElement(entities, "ScenarioObject", {"name": veh})
        vehicle = ET.SubElement(scen_obj, "Vehicle",
            {
                "name": attrib.get("name"),
                "vehicleCategory": "car"
            }
        )

        bound_box = ET.SubElement(vehicle, "BoundingBox")
        center = ET.SubElement(bound_box, "Center",
            {
                "x": "2.0", "y": "0.0", "z": "0.9"
            }
        )
        dim = ET.SubElement(bound_box, "Dimensions",
            {
                "width": "1.61", "length": "4.508", "height": "1.8"
            }
        )

        perform = ET.SubElement(vehicle, "Performance", 
            {
                "maxSpeed": "50.8", "maxDeceleration": "11.5", "maxAcceleration": "11.5"
            }
        )

        axles = ET.SubElement(vehicle, "Axles")
        front_axle = ET.SubElement(axles, "FrontAxle",
            {
                "maxSteering": "0.524", "wheelDiameter": "0.8", "trackWidth": "1.68", "positionX": "2.98", "positionZ": "0.4"
            }
        )
        rear_axle = ET.SubElement(axles, "RearAxle",
            {
                "maxSteering": "0.524", "wheelDiameter": "0.8", "trackWidth": "1.68", "positionX": "0.0", "positionZ": "0.4"
            }
        )

        properties = ET.SubElement(vehicle, "Properties")
        prop = ET.SubElement(properties, "Property",
            {
            "name": veh,
            "value": attrib.get("name")
            }
        )
        pfile = ET.SubElement(properties, "File", {"filepath": attrib.get("model_path")})


    return root



def create_init(root, vehicle_dict, road_points):
    story_board = ET.SubElement(root, "Storyboard")
    init = ET.SubElement(story_board, "Init")
    actions = ET.SubElement(init, "Actions")

    for vehicle, attrib in vehicle_dict.items():
        if attrib.get("start_point") != "None":
            start_point = attrib.get("start_point")
        else:
            start_point = random.choice(road_points)

        private = ET.SubElement(actions, "Private", entityRef=vehicle)
        private_action = ET.SubElement(private, "PrivateAction")
        tel_act = ET.SubElement(private_action, "TeleportAction")
        pos = ET.SubElement(tel_act, "Position")
        ET.SubElement(pos, "WorldPosition", x=str(start_point[0]), y=str(start_point[1]), h="1.570796")

        vehicle_dict[vehicle]["start_point"] = start_point

    return story_board, vehicle_dict



def create_story(story_board, story_name, vehicle_dict, road_points):
    story = ET.SubElement(story_board, "Story", 
        {
        "name": story_name
        }
    )
    act = ET.SubElement(story, "Act", 
        {
            "name": story_name
        }
    )

    for vehicle, attrib in vehicle_dict.items():
        if attrib.get("start_point") != "None":
            start_point = attrib.get("start_point")
        else:
            start_point = random.choice(road_points)

        if attrib.get("end_point") != "None":
            end_point = attrib.get("end_point")
        else:
            end_point = random.choice(road_points)


        manv_group = ET.SubElement(act, "ManeuverGroup",
            {
                "name": f"maneuver_group_{vehicle}",
                "maximumExecutionCount": "1"
            }
        )

        actors = ET.SubElement(manv_group, "Actors", selectTriggeringEntities="false")
        ET.SubElement(actors, "EntityRef", entityRef=vehicle)
        
        manv = ET.SubElement(manv_group, "Maneuver", name=f"maneuver_{vehicle}")
        event = ET.SubElement(manv, "Event", name=f"event_{vehicle}", priority="parallel", maximumExecutionCount="1")
        action = ET.SubElement(event, "Action", name=f"assign_route_{vehicle}")
        priv_act = ET.SubElement(action, "PrivateAction")
        rout_act = ET.SubElement(priv_act, "RoutingAction")
        assign_route = ET.SubElement(rout_act, "AssignRouteAction")
        route = ET.SubElement(assign_route, "Route", name=f"route_{vehicle}", closed="false")
        waypoint1 = ET.SubElement(route, "Waypoint", routeStrategy="shortest")
        pos1 = ET.SubElement(waypoint1, "Position")
        ET.SubElement(pos1, "WorldPosition", x=str(start_point[0]), y=str(start_point[1]))
        waypoint2 = ET.SubElement(route, "Waypoint", routeStrategy="shortest")
        pos2 = ET.SubElement(waypoint2, "Position")
        ET.SubElement(pos2, "WorldPosition", x=str(end_point[0]), y=str(end_point[1]))

        start_trig = ET.SubElement(manv_group, "StartTrigger")
        cond_group = ET.SubElement(start_trig, "ConditionGroup")
        cond = ET.SubElement(cond_group, "Condition", name="act_start", delay="0.0", conditionEdge="none")
        by_val = ET.SubElement(cond, "ByValueCondition")
        ET.SubElement(by_val, "SimulationTimeCondition", value="0.0", rule="greaterThan")


    return story_board