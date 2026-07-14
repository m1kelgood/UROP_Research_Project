import os
import time
import sys
from datetime import datetime
from osc_sumo_converter import read_text, process_text, output_text
from traci_control import traci_control
from no_dynamic_updates import create_full_rou, create_other_sumocfg



# ============================================================================== 
# !!NOTE!! in order for this to work, the following flag must be passed when
# performing the netconvert operations: --output.original-names true
# ==============================================================================



# ==============================================================================
# Set SUMO_HOME environment variable if not already set
# ==============================================================================
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")



# ==============================================================================
# Input and Output File Setup
# ==============================================================================
XOSC_FILE = r"../xosc/atc_t-junction_left_turn_obstacle_left.xosc"
NET_FILE = r"../netconvert_nets/converted_aldenhoven.net.xml"

BASE_NAME = os.path.splitext(os.path.basename(XOSC_FILE))[0]
DATA_FOLDER = f"{BASE_NAME}_SUMO_conversion"
OUT_PATH = r"conversion_output" + "/" + DATA_FOLDER



# ==============================================================================
# Output File Headers
# ==============================================================================
FHEADER = f'''<?xml version="1.0" encoding="UTF-8"?>

<!-- generated on {datetime.now()} by Exclipse SUMO sumo 1.27.0 (VIA .XOSC CONVERSION)
-->
'''



# ==============================================================================
# Traci startup variables
# ==============================================================================
STARTUP = {
    "interface": "sumo-gui",
    "step-length": "0.05",
    "delay": "250",
    "waiting-time-memory": "300",
    "desired_sim_duration": "5000"
}



# ==============================================================================
# Main Function
# ==============================================================================
def main():
    start = time.time()

    openscen_spawn_data, openscen_move_data, catalog_data = read_text(XOSC_FILE)

    sumo_spawn, traci_move, mapping_table = process_text(openscen_spawn_data, openscen_move_data, catalog_data, NET_FILE)

    sumocfg_file, additional_dynamic_commands = output_text(sumo_spawn, OUT_PATH, NET_FILE, BASE_NAME, FHEADER)

    full_rou = create_full_rou(traci_move, BASE_NAME, FHEADER, sumocfg_file)
    create_other_sumocfg(full_rou, NET_FILE, BASE_NAME, FHEADER)

    print(f"SPAWN COMMANDS: {sumo_spawn}\n")
    print(f"MOVE COMMANDS: {traci_move}\n")
    print(f"ADDITIONAL MOVE COMMANDS: {additional_dynamic_commands}\n")

    mid = time.time() 

    print(f"\nConversion complete. Time taken: {mid - start:.4f} seconds")

    traci_control(additional_dynamic_commands, traci_move, mapping_table, OUT_PATH, sumocfg_file, STARTUP)

    end = time.time()

    print(f"\nSimulation Complete. Time taken: {end - mid:.4f} seconds")



if __name__ == "__main__":
    main()