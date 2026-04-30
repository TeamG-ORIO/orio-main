import tkinter as tk
from tkinter import ttk, messagebox
import sqlite3
import os
from datetime import datetime
import argparse  # NEW: Imported for command-line arguments

# ==========================================
# Configuration
# ==========================================
DB_NAME = 'medical_inventory.db'
# DB_NAME = "test.db"

POLL_INTERVAL_MS = 1500  # Refresh the GUI every 1.5 seconds

class InventoryDashboard(tk.Tk):
    # NEW: Added show_new_items parameter
    def __init__(self, show_new_items=True):
        super().__init__()
        self.title("Robotic Capstone: Inventory Dashboard")
        self.geometry("900x500")
        
        self.show_new_items = show_new_items  # Store the flag state
        
        # Define "this session" as any item labeled after this GUI was launched
        self.session_start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.expired_items_cache = []

        self.setup_ui()
        self.poll_database()

    def setup_ui(self):
        # --- Top Frame: Alert Bar ---
        top_frame = tk.Frame(self, pady=10)
        top_frame.pack(side=tk.TOP, fill=tk.X, padx=10)

        self.alert_btn = tk.Button(
            top_frame, 
            text="Checking expiration dates...", 
            font=("Arial", 12, "bold"), 
            command=self.show_expired_items
        )
        self.alert_btn.pack(side=tk.RIGHT, ipadx=10, ipady=5)

        tk.Label(top_frame, text="Real-Time Database Viewer", font=("Arial", 16, "bold")).pack(side=tk.LEFT)

        self.totals_label = tk.Label(top_frame, text="Shelf / Total: 0/0", font=("Arial", 12, "italic"), fg="#333333")
        self.totals_label.pack(side=tk.LEFT, padx=20)

        # --- Main Content Frame ---
        content_frame = tk.Frame(self)
        content_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=5)

        # NEW: Conditionally render the left frame
        if self.show_new_items:
            # --- Left Panel: Newly Labeled Items ---
            left_frame = tk.Frame(content_frame)
            left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

            tk.Label(left_frame, text="Newly Labeled (This Session)", font=("Arial", 12)).pack(anchor="w")
            
            columns_new = ("Item Name", "RFID UID")
            self.tree_new = ttk.Treeview(left_frame, columns=columns_new, show="headings", height=15)
            for col in columns_new:
                self.tree_new.heading(col, text=col)
                self.tree_new.column(col, width=150, anchor="w")
            self.tree_new.pack(fill=tk.BOTH, expand=True)

        # --- Right Panel: On Shelf Inventory ---
        right_frame = tk.Frame(content_frame)
        
        # NEW: Adjust packing based on whether the left frame exists
        if self.show_new_items:
            right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        else:
            right_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=0)

        tk.Label(right_frame, text="Current Shelf Inventory", font=("Arial", 12)).pack(anchor="w")
        
        columns_shelf = ("Item Name", "Count")
        self.tree_shelf = ttk.Treeview(right_frame, columns=columns_shelf, show="headings", height=15)
        for col in columns_shelf:
            self.tree_shelf.heading(col, text=col)
        self.tree_shelf.column("Item Name", width=200, anchor="w")
        self.tree_shelf.column("Count", width=80, anchor="center")
        self.tree_shelf.pack(fill=tk.BOTH, expand=True)

    def connect_db_readonly(self):
        if not os.path.exists(DB_NAME):
            return None
        db_uri = f"file:{DB_NAME}?mode=ro"
        return sqlite3.connect(db_uri, uri=True)

    def poll_database(self):
        conn = self.connect_db_readonly()
        
        if conn:
            try:
                cursor = conn.cursor()

                # 1. Fetch Newly Labeled Items
                cursor.execute('''
                    SELECT item_name, rfid_uid 
                    FROM medical_supplies 
                    WHERE date_labeled >= ? 
                    ORDER BY date_labeled DESC
                ''', (self.session_start_time,))
                new_items = cursor.fetchall()

                # 2. Fetch Shelf Inventory Counts
                cursor.execute('''
                    SELECT item_name, COUNT(*) 
                    FROM medical_supplies 
                    WHERE is_on_shelf = 1 
                    GROUP BY item_name
                ''')
                shelf_counts = cursor.fetchall()

                # 3. Fetch Expired Items on the Shelf
                today = datetime.now().strftime("%Y-%m-%d")
                cursor.execute('''
                    SELECT rfid_uid, item_name, expiration_date 
                    FROM medical_supplies 
                    WHERE is_on_shelf = 1 AND expiration_date < ?
                ''', (today,))
                self.expired_items_cache = cursor.fetchall()

                # 4. Fetch Total Database Count
                cursor.execute('SELECT COUNT(*) FROM medical_supplies')
                total_db_count = cursor.fetchone()[0] - 1

                # 5. Fetch Total Shelf Count
                cursor.execute('SELECT COUNT(*) FROM medical_supplies WHERE is_on_shelf = 1')
                total_shelf_count = cursor.fetchone()[0]

                # Update the GUI with the fetched data
                self.update_tables(new_items, shelf_counts)
                self.update_totals_label(total_shelf_count, total_db_count)
                self.update_alert_button()

            except sqlite3.Error as e:
                print(f"Database Read Error: {e}")
            finally:
                conn.close()

        self.after(POLL_INTERVAL_MS, self.poll_database)

    def update_tables(self, new_items, shelf_counts):
        # NEW: Only clear and update tree_new if it was created
        if self.show_new_items:
            for item in self.tree_new.get_children():
                self.tree_new.delete(item)
            for item_name, rfid in new_items:
                self.tree_new.insert("", tk.END, values=(item_name, rfid))
            
        # Update shelf items normally
        for item in self.tree_shelf.get_children():
            self.tree_shelf.delete(item)
        for item_name, count in shelf_counts:
            self.tree_shelf.insert("", tk.END, values=(item_name, count))

    def update_totals_label(self, shelf_count, db_count):
        self.totals_label.config(text=f"Total Items on Shelf / DB: {shelf_count}/{db_count}")

    def update_alert_button(self):
        expired_count = len(self.expired_items_cache)
        if expired_count > 0:
            self.alert_btn.config(
                text=f"⚠️ ALERT: {expired_count} Expired Items on Shelf!",
                bg="red", 
                fg="white",
                activebackground="darkred",
                activeforeground="white",
                state=tk.NORMAL
            )
        else:
            self.alert_btn.config(
                text="✅ All Shelf Items Valid", 
                bg="lightgreen", 
                fg="black",
                state=tk.DISABLED
            )

    def show_expired_items(self):
        if not self.expired_items_cache:
            return

        popup = tk.Toplevel(self)
        popup.title("Expired Items Action Required")
        popup.geometry("500x300")

        tk.Label(popup, text="Please remove the following items from the shelf:", font=("Arial", 11, "bold")).pack(pady=10)

        columns = ("RFID UID", "Item Name", "Expiration Date")
        tree = ttk.Treeview(popup, columns=columns, show="headings")
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, width=150, anchor="center")
        tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        for item in self.expired_items_cache:
            tree.insert("", tk.END, values=item)

        tk.Button(popup, text="Close", command=popup.destroy).pack(pady=5)

if __name__ == "__main__":
    # NEW: Implement argument parsing
    parser = argparse.ArgumentParser(description="Launch the RFID Inventory Dashboard.")
    parser.add_argument(
        "--hide-new", 
        action="store_true", 
        help="Hide the 'Newly Labeled' panel and expand the Shelf Inventory panel."
    )
    args = parser.parse_args()

    # Pass the inverted boolean flag (if --hide-new is passed, show_new is False)
    app = InventoryDashboard(show_new_items=not args.hide_new)
    app.mainloop()