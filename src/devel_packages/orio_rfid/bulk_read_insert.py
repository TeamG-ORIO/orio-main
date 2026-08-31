import time
import sqlite3
import sys
import argparse
from datetime import datetime
from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT

# ==========================================
# Configuration
# ==========================================
READER_IP = "192.168.0.20" 
DB_NAME = 'medical_inventory.db'

# ==========================================
# Database Setup
# ==========================================
def setup_database():
    """Initializes the SQLite database with the medical_supplies schema."""
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS medical_supplies (
            rfid_uid TEXT PRIMARY KEY,
            item_name TEXT,
            expiration_date TEXT,
            date_labeled TEXT,
            last_seen TEXT,
            is_on_shelf INTEGER DEFAULT 0
        )
    ''')
    conn.commit()
    return conn

# ==========================================
# Global State
# ==========================================
scanned_epcs = set()
total_raw_reads = 0

# ==========================================
# LLRP Callback
# ==========================================
def tag_report_cb(reader, tag_reports):
    """Tracks unique tags and prints new discoveries in real-time."""
    global total_raw_reads
    total_raw_reads += len(tag_reports)
    
    for tag in tag_reports:
        epc = tag.get('EPC-96', tag.get('EPCData', b'Unknown'))
        
        if isinstance(epc, bytes):
            epc = str(epc).upper()[2:-1]
            
        if epc not in scanned_epcs:
            scanned_epcs.add(epc)
            rssi = tag.get('PeakRSSI', 'N/A')
            print(f"New Tag Found -> UID: {epc} | RSSI: {rssi} dBm")

# ==========================================
# Main Execution
# ==========================================
def main():
    # 1. Setup Command Line Argument Parsing
    parser = argparse.ArgumentParser(description="Capstone Mass RFID Registration")
    parser.add_argument("-n", "--name", required=True, help="Item Name (e.g., '50ml Saline Syringe')")
    parser.add_argument("-e", "--exp", required=True, help="Expiration Date (YYYY-MM-DD)")
    parser.add_argument("-t", "--time", type=int, default=5, help="Scan duration in seconds (default: 5)")
    
    args = parser.parse_args()
    
    item_name = args.name.strip()
    exp_date = args.exp.strip()
    scan_duration = args.time

    print("\n--- Capstone Mass RFID Registration ---")
    print(f"Target Item: {item_name}")
    print(f"Expiration:  {exp_date}")
    print(f"Duration:    {scan_duration} seconds")

    # 2. Configure the Reader
    print(f"\nConnecting to Impinj reader at {READER_IP}...")
    config_dict = {
        'antennas': [4],
        'tx_power_dbm': 10.0,  
        'tag_content_selector': {
            'EnablePeakRSSI': True,
            'EnableAntennaID': False,
            'EnableTagSeenCount': False,
        }
    }
    
    config = LLRPReaderConfig(config_dict)
    reader = LLRPReaderClient(READER_IP, LLRP_DEFAULT_PORT, config)
    reader.add_tag_report_callback(tag_report_cb)
    
    # 3. Run the Time-Bounded Scan
    try:
        reader.connect()
        print(f"\n[SCAN ACTIVE] Please place all '{item_name}' items on the pad.")
        print(f"Scanning for {scan_duration} seconds...")
        time.sleep(scan_duration)
    except KeyboardInterrupt:
        print("\nScan interrupted.")
    except Exception as e:
        print(f"\nError communicating with reader: {e}")
        sys.exit(1)
    finally:
        print("\n[SCAN COMPLETE] Disconnecting from reader...")
        reader.disconnect()

    print("Waiting 1 seconds for the network buffer to flush...")
    time.sleep(1)

    # 4. Process the Data Post-Scan
    total_found = len(scanned_epcs)
    print("\n" + "="*40)
    print("--- SCAN SUMMARY ---")
    print(f"Total Raw Tag Reads: {total_raw_reads}")
    print(f"Total Unique Items:  {total_found}")
    print("="*40 + "\n")

    if total_found > 0:
        db_conn = setup_database()
        cursor = db_conn.cursor()
        added_count = 0
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        for epc in scanned_epcs:
            cursor.execute("SELECT rfid_uid FROM medical_supplies WHERE rfid_uid = ?", (epc,))
            if cursor.fetchone() is None:
                # We changed the second 'timestamp' to 'None' below
                cursor.execute('''
                    INSERT INTO medical_supplies 
                    (rfid_uid, item_name, expiration_date, date_labeled, last_seen, is_on_shelf) 
                    VALUES (?, ?, ?, ?, ?, ?)
                ''', (epc, item_name, exp_date, timestamp, None, 0))
                added_count += 1
        
        db_conn.commit()
        db_conn.close()
        
        print(f"Success! Registered {added_count} NEW '{item_name}' items to the database.")
        if added_count < total_found:
            print(f"({total_found - added_count} tags were already registered and skipped).")
    else:
        print("No tags were registered. Please ensure items are within the antenna's range.")

if __name__ == '__main__':
    main()