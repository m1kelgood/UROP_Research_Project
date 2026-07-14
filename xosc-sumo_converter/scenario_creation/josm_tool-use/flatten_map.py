import xml.etree.ElementTree as ET

####################################################################################################
# Initially in the conversion of the .osm map to .xodr, there are elevations incorporated.
# However, to accommodate for the routing approach in create_scenario.py, this file 'flattens'
# The map to ensure that the route has no elevation
####################################################################################################


INPUT = "campus.xodr"
OUTPUT = "flat_campus.xodr"


if __name__ == "__main__":
    tree = ET.parse(INPUT)
    root = tree.getroot()


    for child in root:
        if child.tag == "road":
            for elem in child:
                if elem.tag == "elevationProfile":
                    for elev in elem:
                        elev.set("s", "0.0")
                        elev.set("a", "0.0")
                        elev.set("b", "0.0")
                        elev.set("c", "0.0")
                        elev.set("d", "0.0")

                elif elem.tag == "lateralProfile":
                    for elev in elem:
                        if elev.tag == "superelevation":
                            elev.set("a", "0.0")
                            elev.set("b", "0.0")
                            elev.set("c", "0.0")
                            elev.set("d", "0.0")


tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)