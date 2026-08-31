import mercury
import sys

# --- CONFIGURATION ---
# Change to 'tmr:///dev/ttyACM0' or 'tmr:///dev/ttyUSB1' if it fails to connect
PORT = "tmr:///dev/ttyUSB0" 
# 'NA2' or 'NA3' for North America, 'EU3' for Europe
REGION = "NA" 
# Power in centi-dBm (1000 = 10 dBm). Max is 2700, but 1000 is very safe for USB.
SAFE_POWER = 1000 
# ---------------------

def run_safe_scan():
    print(f"Attempting to connect to M7E Reader on {PORT}...")
    
    try:
        # Initialize the reader
        reader = mercury.Reader(PORT)
        
        # 1. Set the region (Mandatory before scanning)
        reader.set_region(REGION)
        print(f"Region set to: {REGION}")

        # 2. SET THE SAFE POWER LIMITS
        # The syntax is a list of tuples: [(antenna_port, power_in_cdbm)]
        reader.set_read_powers([(1, SAFE_POWER)])
        
        # Optional but good practice: set write power to match, 
        # in case you try writing to tags later.
        try:
             reader.set_write_powers([(1, SAFE_POWER)])
        except Exception:
             pass # Some firmware versions ignore write power setting, we can safely pass
             
        # Verify the power was set correctly
        current_power = reader.get_read_powers()[0][1]
        print(f"Hardware transmit power verified at: {current_power / 100} dBm")

        # 3. Configure the Read Plan
        # We are using Antenna 1 and scanning for Gen2 tags
        reader.set_read_plan([1], "GEN2")

        # 4. Execute the Scan
        print("\nScanning for 3 seconds. Bring a UHF RFID tag near the antenna...")
        
        # Read for 3000 milliseconds
        tags = reader.read(3000)

        # 5. Process the Results
        print("\n--- SCAN RESULTS ---")
        if not tags:
            print("No tags detected. Try moving the tag closer or checking orientation.")
        else:
            for tag in tags:
                # epc is a bytearray, convert to hex string for readability
                epc_string = tag.epc.hex().upper()
                print(f"Tag EPC: {epc_string} | RSSI: {tag.rssi} dBm | Read Count: {tag.read_count}")
        print("--------------------\n")

    except Exception as e:
        print(f"\n[ERROR] Something went wrong: {e}")
        print("Troubleshooting tips:")
        print("1. Did you flip the hardware switch to 'USB'?")
        print("2. Are you using the correct /dev/ttyUSBx port?")
        print("3. Do you have permission to access the port? (Did you add your user to the 'dialout' group?)")
        sys.exit(1)

if __name__ == "__main__":
    run_safe_scan()