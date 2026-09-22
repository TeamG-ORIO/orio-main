#!/usr/bin/env python3
import os
import shutil
import rospy
import sqlite3
import threading
import time
from datetime import datetime

from multiprocessing import Process, Queue as MpQueue

# Impinj imports
from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT

# Mercury imports
import mercury

# Custom message
from custom_msgs.srv import AddLabeledItem, AddLabeledItemResponse

# --- CONFIGURATION ---
IMPINJ_IP = "192.168.0.20"

MERCURY_PORT = "tmr:///dev/ttyUSB0"
MERCURY_REGION = "NA"
MERCURY_POWER = 1000

# Resolve paths relative to this file so it works regardless of CWD.
_HERE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(_HERE, "medical_inventory.db")            # mutable runtime copy (git-ignored)
SEED_DB_PATH = os.path.join(_HERE, "seed_medical_inventory.db")  # canonical catalog seed (tracked)

# Seed the runtime DB from the committed seed on first run (fresh clone has no runtime DB).
if not os.path.exists(DB_PATH) and os.path.exists(SEED_DB_PATH):
    shutil.copy(SEED_DB_PATH, DB_PATH)
# ---------------------

# ==========================================
# ISOLATED IMPINJ PROCESS (Runs ONCE and dies cleanly)
# ==========================================
def run_impinj_process(queue_out):
    def _dumb_callback(reader, tag_reports):
        for tag in tag_reports:
            queue_out.put(tag)

    config_dict = {
        'antennas': [1, 4],
        'tx_power_dbm': 30.0, 
        'tag_content_selector': {
            'EnablePeakRSSI': True,
            'EnableAntennaID': True,
            'EnableTagSeenCount': True,
        }
    }
    
    try:
        config = LLRPReaderConfig(config_dict)
        client = LLRPReaderClient(IMPINJ_IP, LLRP_DEFAULT_PORT, config)
        client.add_tag_report_callback(_dumb_callback)
        
        # Connect, read for 2 seconds, and force flush
        client.connect()
        time.sleep(2.0)
        client.disconnect() # This flushes the buffer and stops the Twisted reactor
        time.sleep(1.0)
        
    except Exception as e:
        print(f"[IMPINJ PROCESS ERROR] {e}", flush=True)
    # The function ends here, terminating the multiprocessing.Process cleanly


# ==========================================
# MAIN ROS MANAGER
# ==========================================
class SimpleRFIDManager:
    def __init__(self):
        self.add_new_item = False
        self.pending_item_name = ""
        self.pending_expiration = ""
        self.service_completed_event = threading.Event()
        self.service_success = False
        self.service_message = ""
        
        self.tag_queue = MpQueue()
        self.impinj_process = None # Tracks the current "ping" process

        print("[INIT] Initializing ROS Node...", flush=True)
        rospy.init_node('rfid_db_manager')

        self.add_srv = rospy.Service('/inventory/add_item', AddLabeledItem, self.handle_add_item)

        rospy.loginfo("Simple RFID Manager fully initialized. Starting main loop...")
        self.run_main_loop()

    # ==========================================
    # ROS SERVICE CALLBACK
    # ==========================================
    def handle_add_item(self, req):
        rospy.loginfo(f"ROS Service Call received for: {req.item_name}")
        
        self.pending_item_name = req.item_name
        self.pending_expiration = req.expiration_date
        
        self.service_completed_event.clear()
        self.add_new_item = True
        self.service_completed_event.wait()
        
        return AddLabeledItemResponse(success=self.service_success, message=self.service_message)

    # ==========================================
    # HELPER: SPAWN IMPINJ
    # ==========================================
    def spawn_impinj_ping(self):
        """Spawns a fresh Impinj process for a 2-second radar ping."""
        self.impinj_process = Process(target=run_impinj_process, args=(self.tag_queue,))
        self.impinj_process.daemon = True
        self.impinj_process.start()

    # ==========================================
    # THE SINGLE MAIN LOOP
    # ==========================================
    def run_main_loop(self):
        conn = sqlite3.connect(DB_PATH)
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

        rate = rospy.Rate(1) 

        while not rospy.is_shutdown():
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # --- STATE: TRUE (Process ROS Service Request via Mercury) ---
            if self.add_new_item:
                rospy.loginfo("State: LABELING. Activating Mercury Reader...")
                
                # If an Impinj process is currently running, let it die naturally before we label
                if self.impinj_process and self.impinj_process.is_alive():
                    self.impinj_process.join(timeout=3.0)

                try:
                    reader = mercury.Reader(MERCURY_PORT)
                    reader.set_region(MERCURY_REGION)
                    reader.set_read_powers([(1, MERCURY_POWER)])
                    reader.set_read_plan([1], "GEN2")
                    
                    tags = reader.read(3000) 
                    
                    if tags:
                        tag = max(tags, key=lambda x:x.read_count)
                        epc_bytes = bytes(tag.epc)
                        if len(epc_bytes) > 16:
                            try:
                                epc_string = epc_bytes.decode('utf-8').upper()
                            except UnicodeDecodeError:
                                epc_string = epc_bytes.hex().upper()
                        else:
                            epc_string = epc_bytes.hex().upper()
                            
                        rospy.loginfo(f"Scanned new label: {epc_string}")
                        
                        try:
                            cursor.execute('''
                                INSERT INTO medical_supplies (rfid_uid, item_name, expiration_date, date_labeled, last_seen, is_on_shelf)
                                VALUES (?, ?, ?, ?, NULL, 0)
                            ''', (epc_string, self.pending_item_name, self.pending_expiration, current_time))
                            conn.commit()
                            
                            self.service_success = True
                            self.service_message = "Item added successfully."
                            rospy.loginfo("Database insert successful.")
                            
                        except sqlite3.IntegrityError:
                            self.service_success = False
                            self.service_message = "RFID already exists in database."
                            rospy.logwarn(self.service_message)
                    else:
                        self.service_success = False
                        self.service_message = "No tag detected by Mercury reader."
                        rospy.logwarn(self.service_message)
                        
                except Exception as e:
                    self.service_success = False
                    self.service_message = f"Hardware Error: {e}"
                    rospy.logerr(self.service_message)

                # Reset state flags
                self.add_new_item = False
                self.service_completed_event.set()
                
                # Flush the queue to discard any tags read right before labeling started
                while not self.tag_queue.empty():
                    try:
                        self.tag_queue.get_nowait()
                    except Exception:
                        break

            # --- STATE: FALSE (Normal Shelf Scanning via Impinj) ---
            else:
                # 1. Trigger the radar ping if the previous one finished
                if self.impinj_process is None or not self.impinj_process.is_alive():
                    self.spawn_impinj_ping()

                scanned_this_second = set()
                
                # 2. Drain everything currently in the multiprocessing pipe
                while not self.tag_queue.empty():
                    try:
                        raw_tag = self.tag_queue.get_nowait()
                        epc = raw_tag.get('EPC-96', raw_tag.get('EPCData', b'Unknown'))
                        
                        if isinstance(epc, bytes):
                            epc_string = str(epc).upper()[2:-1]
                            scanned_this_second.add(epc_string)
                    except Exception:
                        break

                # 3. If we found valid tags in the queue, update the database
                if scanned_this_second:
                    rospy.loginfo(f"Impinj Heartbeat: Updating {len(scanned_this_second)} items on shelf.")
                    batch_data = [(current_time, rfid) for rfid in scanned_this_second]
                    
                    try:
                        cursor.executemany('''
                            UPDATE medical_supplies 
                            SET last_seen = ?, is_on_shelf = 1 
                            WHERE rfid_uid = ?
                        ''', batch_data)
                        conn.commit()
                    except Exception as e:
                        rospy.logerr(f"Database heartbeat update failed: {e}")
                        
                # 4. Always run the flush routine to mark old items as missing
                try:
                    cursor.execute('''
                        UPDATE medical_supplies
                        SET is_on_shelf = 0
                        WHERE is_on_shelf = 1 
                        AND strftime('%s', 'now', 'localtime') - strftime('%s', last_seen) > 5
                    ''')
                    conn.commit()
                except Exception as e:
                    rospy.logerr(f"Database flush failed: {e}")

            rate.sleep()

        conn.close()

if __name__ == '__main__':
    try:
        SimpleRFIDManager()
    except rospy.ROSInterruptException:
        pass