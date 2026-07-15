import os
import argparse
import logging
from pathlib import Path
import time
import sys
from datetime import datetime
from osc_sumo_converter import read_text, process_text, output_text
from traci_control import traci_control
from no_dynamic_updates import create_full_rou, create_other_sumocfg



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
DEFUALT_XOSC_FILE = Path("../xosc/highway_merge.xosc")
DEFUALT_NET_FILE = Path("../netconvert_nets/soderleden.net.xml")



# ==============================================================================
# Output File Headers
# ==============================================================================
FHEADER = f'''<?xml version="1.0" encoding="UTF-8"?>

<!-- generated on {datetime.now()} by Exclipse SUMO sumo 1.27.0 (VIA .XOSC CONVERSION)
-->
'''



# ==============================================================================
# Main Function
# ==============================================================================
def main():

    # Program header
    print(
        f"xosc-sumo scenario transfer progam\n{'-'*80}\n"
        "Note: when using this program, if modifying the default arguments, ensure that the xosc "
        "and netfilesare compatible with one another. Also, when providing an xodr file to netconvert, "
        f"you must pass the command '--output.original-names true' for the lane/edge mapping to work.\n{'-'*80}\n"
    )


    # CLI arg options
    parser = argparse.ArgumentParser(description="Argscript for control of xosc-sumo file converter")
    parser.add_argument("--xosc", type=Path, default=DEFUALT_XOSC_FILE, help=".xosc file to convert")

    parser.add_argument("--net", type=Path, default=DEFUALT_NET_FILE,
                        help=".net file that has been converted from the matching .xodr using netconvert")

    parser.add_argument("--interface", type=str, default="sumo-gui", help="Enable SUMO gui rendering")
    parser.add_argument("--step_length", type=float, default=0.05, help="SUMO simulation step length")
    parser.add_argument("--delay", type=int, default=250, help="SUMO simulation delay")
    parser.add_argument("--sim_dur", type=int, default=1000, help="SUMO simulation duration")
    parser.add_argument("--debug", type=bool, default=False, help="Debug for developent")
    parser.add_argument("--stop_at_full_rou", type=bool, default=False, help="If only wanting full rou and avoiding traci simulation")
    args = parser.parse_args()


    # Setup output folders
    base_name = os.path.splitext(os.path.basename(args.xosc))[0]
    data_folder = f"{base_name}_SUMO_conversion"
    out_path = f"conversion_output/{data_folder}"


    # Traci control startup commands
    startup = {
        "interface": args.interface, "step-length": args.step_length, "delay": args.delay,
        "desired_sim_duration": args.sim_dur
    }


    # Record time for optimization purposes
    start = time.time()


    # Read text
    openscen_spawn_data, openscen_move_data, catalog_data = read_text(args.xosc)
    read_text_time = time.time()

    print(f"Finished parsing the xosc file in {read_text_time-start:.4f} seconds\n")
    if args.debug:
        print(
            f"OpenSCENARIO Spawn Data:\n{'-'*80}\n{openscen_spawn_data}\n{'-'*80}\n\n"
            f"OpenSCENARIO Move Data:\n{'-'*80}{openscen_move_data}\n{'-'*80}\n\n"
            f"Catalog Data:\n{'-'*80}\n{catalog_data}\n{'-'*80}\n\n"
        )
    

    # Process text (if necessary) to get in SUMO acceptable format
    sumo_spawn, traci_move, mapping_table = process_text(openscen_spawn_data, openscen_move_data, catalog_data, args.net)
    process_text_time = time.time()

    print(f"Finished processing the commands in {process_text_time-read_text_time:.4f} seconds\n")
    if args.debug:
        print(
            f"SUMO spawn commands:\n{'-'*80}\n{sumo_spawn}\n{'-'*80}\n\n"
            f"Traci move commands:\n{'-'*80}\n{traci_move}\n{'-'*80}\n\n"
            f"Edge/lane mapping table:\n{'-'*80}\n{mapping_table}\n{'-'*80}\n\n"
        )


    # Output text through the creation of .rou and .sumocfg files
    sumocfg_file, additional_dynamic_commands = output_text(sumo_spawn, out_path, args.net, base_name, FHEADER)
    full_rou = create_full_rou(traci_move, base_name, FHEADER, sumocfg_file)
    create_other_sumocfg(full_rou, args.net, base_name, FHEADER)
    output_text_time = time.time()

    print(
        f"Finished the output of SUMO files in {output_text_time-process_text_time:.4f} seconds\n"
        f"Find the traci dynamic .sumocfg file at {sumocfg_file}\n"
        f"Find the full static .rou file (still under development) at {full_rou}\n"
    )

    if args.debug:
        print(
            f"Need to investigate in greater detail if additional dynamic commands is useful at all"
            f"\n{'-'*80}\n{tradditional_dynamic_commandsaci_move}\n{'-'*80}\n\n"
        )

    print(f"xosc conversion complete. Total time taken: {output_text_time-start:.4f}\n")

    if args.stop_at_full_rou:
        return

    else:
        print("Beginning traci control\n")

    # Traci dynamic control
    traci_begin = time.time()
    traci_control(additional_dynamic_commands, traci_move, mapping_table, out_path, sumocfg_file, startup)
    end = time.time()

    print(f"\nSimulation Complete. Time taken: {end - traci_begin:.4f} seconds")



if __name__ == "__main__":
    main()