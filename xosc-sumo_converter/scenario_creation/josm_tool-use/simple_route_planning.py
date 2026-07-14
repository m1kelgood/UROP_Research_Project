#!/usr/bin/env python3

import argparse
from cProfile import label

parser = argparse.ArgumentParser(description='Load Lanelet2 map .osm file and plan a simple route.')
parser.add_argument('file', metavar='file', type=str, help='the Lanelet2 map .osm file')
parser.add_argument('--lat-origin', type=float, default=50.90803226101158, help='Latitude origin for the map projection (default: 0.0)')
parser.add_argument('--lon-origin', type=float, default=6.226988816460529, help='Longitude origin for the map projection (default: 0.0)')
parser.add_argument('--lat-target', type=float, default=50.90752314525801, help='Latitude of target vehicle')
parser.add_argument('--lon-target', type=float, default=6.225838783125758, help='Longitude of target vehicle')
parser.add_argument('--lat-destination', type=float, default=50.90818360647998, help='Latitude of destination')
parser.add_argument('--lon-destination', type=float, default=6.226135760601868, help='Longitude of destination')

import lanelet2
from lanelet2.projection import UtmProjector

import matplotlib.pyplot as plt


def load_map(map_file, projector):
    try:
        map, err = lanelet2.io.loadRobust(map_file, projector)
        print("Map loaded successfully.")
        return map
    except Exception as e:
        pass
        print(f"Failed to load map: {e}")

def plot_lanelets(map):
    fig, ax = plt.subplots()
    for lanelet in map.laneletLayer:
        left = lanelet.leftBound
        right = lanelet.rightBound
        left_xy = [(pt.x, pt.y) for pt in left]
        right_xy = [(pt.x, pt.y) for pt in right]
        ax.plot(*zip(*left_xy), color='blue', linewidth=2, label='Left Bound')
        ax.plot(*zip(*right_xy), color='red', linewidth=2, label='Right Bound')

    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())
    plt.title('Lanelet2 Boundaries')
    plt.savefig(f"matplot_files/lanelet_map.png")
    return ax

def plot_target_and_destination(ax, projector, target_latlon, destination_latlon):
    
    # Convert lat-lon to map coordinates using the projector
    target_pos = projector.forward(lanelet2.core.GPSPoint(target_latlon[0], target_latlon[1], 0))
    destination_pos = projector.forward(lanelet2.core.GPSPoint(destination_latlon[0], destination_latlon[1], 0))

    ax.plot(target_pos.x, target_pos.y, marker='o', color='green', markersize=10, label='Target')
    ax.plot(destination_pos.x, destination_pos.y, marker='*', color='orange', markersize=12, label='Destination')
    return ax, target_pos, destination_pos

def find_nearest_lanelet(ll_map, pos, rules):
    """
    Function that returns the lanelet-object nearest to the input position
    Arguments:
    ll_map -- Lanelet2 map-object
    pos -- `BasicPoint3d` position where lanelet objects shall be found
    rules -- Lanelet2 traffic-rules object
    
    Returns:
    ll -- lanelet-object nearest to the given input position
    """
    lanelets=lanelet2.geometry.findWithin3d(ll_map.laneletLayer, pos, 5.0)
    if(len(lanelets)>0):
        for ll in lanelets:
            if(rules.canPass(ll[1])):
                return ll[1]
        print("No Lanelet found which can be used by a vehicle!")
        return
    else:
        print("No Lanelets found at given map-position!")
        return
    
def lanelets2path(lanelet_path):
    """
    Function that derives a x-y-path from a given lanelet-sequence
    Arguments:
    lanelet_path -- list of lanelet objects consecuting the shortest path
    
    Returns:
    x -- list of x-coordinates of the resulting path
    y -- list of y-coordinates of the resulting path
    """
    x=[]
    y=[]
    for ll in lanelet_path:
        cl = ll.centerline
        for p in cl:
            x.append(p.x)
            y.append(p.y)
    return x, y

if __name__ == '__main__':
    args = parser.parse_args()
    projector = UtmProjector(lanelet2.io.Origin(args.lat_origin, args.lon_origin))
    ll2_map = load_map(args.file, projector)
    ax = plot_lanelets(ll2_map)
    # Example target and destination positions (in lat-lon)
    target_latlon = (args.lat_target, args.lon_target)
    destination_latlon = (args.lat_destination, args.lon_destination)
    ax, target_pos, destination_pos = plot_target_and_destination(ax, projector, target_latlon, destination_latlon)

    trafficRules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
    routingGraph = lanelet2.routing.RoutingGraph(ll2_map, trafficRules)
    target_lanelet = find_nearest_lanelet(ll2_map, target_pos, trafficRules)
    destination_lanelet = find_nearest_lanelet(ll2_map, destination_pos, trafficRules)
    print(f"Target Lanelet ID: {target_lanelet.id if target_lanelet else 'None'}")
    print(f"Destination Lanelet ID: {destination_lanelet.id if destination_lanelet else 'None'}")

    route1 = routingGraph.getRoute(target_lanelet, destination_lanelet)
    route2 = routingGraph.getRoute(target_lanelet.invert(), destination_lanelet)
    route3 = routingGraph.getRoute(target_lanelet, destination_lanelet.invert())
    route4 = routingGraph.getRoute(target_lanelet.invert(), destination_lanelet.invert())
    # Select the route with the lowest cost among the four possible routes
    routes = [route1, route2, route3, route4]
    min_cost = float('inf')
    route = None
    for r in routes:
        if r is not None:
            cost = r.length2d()
            print(f"Route cost: {cost}")
            if cost < min_cost:
                min_cost = cost
                route = r
    print(f"Selected route with cost: {min_cost}")
    if (route is not None):
        ll_path = route.shortestPath()
    else:
        print("Route planning is not possible!")
        exit(1)

    # Add the shortest path-centerline to the plot
    x, y = lanelets2path(ll_path)
    ax.plot(x, y, color='cyan', linewidth=3, label='Shortest Path')

    plt.show()