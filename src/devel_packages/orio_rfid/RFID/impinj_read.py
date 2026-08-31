import time
import logging
from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT

# --- CONFIGURATION ---
# Replace with your reader's actual IP address or hostname
READER_IP = "192.168.0.20" 
# ---------------------

# 1. Global variables to track our counts
scanned_epcs = set()
total_raw_reads = 0

def tag_report_cb(reader, tag_reports):
    """This function is called by the background thread every time tags are seen"""
    global total_raw_reads
    
    # Track every single read the antenna makes (including duplicates)
    total_raw_reads += len(tag_reports)
    
    for tag in tag_reports:
        # Extract the tag data
        epc = tag.get('EPC-96', tag.get('EPCData', b'Unknown'))
        
        # Convert the raw bytes to a readable hex string
        if isinstance(epc, bytes):
            epc = str(epc).upper()[2:-1]
            
        # 2. Only print the tag if we haven't seen it yet
        if epc not in scanned_epcs:
            scanned_epcs.add(epc) # Add to our unique set
            
            rssi = tag.get('PeakRSSI', 'N/A')
            count = tag.get('TagSeenCount', 1)
            
            print(f"New Tag Found -> EPC: {epc} | RSSI: {rssi} dBm | Count: {count}")

def start_scan():
    print(f"Attempting to connect to Impinj R420 at {READER_IP}...")
    
    # 1. Initialize Reader Configuration
    config_dict = {
        'antennas': [4],
        'tx_power_dbm': 15.0,  
        'tag_content_selector': {
            'EnablePeakRSSI': True,
            'EnableAntennaID': True,
            'EnableTagSeenCount': True,
        }
    }
    config = LLRPReaderConfig(config_dict)
    
    # 2. Create the Reader Client
    reader = LLRPReaderClient(READER_IP, LLRP_DEFAULT_PORT, config)
    
    # 3. Attach our callback function
    reader.add_tag_report_callback(tag_report_cb)
    
    # 4. Connect to the reader 
    reader.connect()
    print("Scanning for tags for 5 seconds...\n")
    
    try:
        # Sleep the main thread while the sllurp background thread listens for tags
        time.sleep(5)
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    finally:
        # Cleanly stop the inventory and disconnect
        print("\nScan complete. Shutting down connection.")
        reader.disconnect()
        
        # 3. Print the final exact counts
        print("\n" + "="*40)
        print("--- SCAN SUMMARY ---")
        print(f"Total Raw Tag Reads (including duplicates): {total_raw_reads}")
        print(f"Total UNIQUE Tags Scanned and Printed:      {len(scanned_epcs)}")
        print("="*40 + "\n")

if __name__ == '__main__':
    start_scan()