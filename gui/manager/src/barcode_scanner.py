import sys
import os
sys.path.append('./db/src')

import json
from DatabaseManager import DatabaseManager
from datetime import datetime

class BarcodeScanner:
    def __init__(self):
        self.db_manager = DatabaseManager(host='localhost')
        self.db_manager.connect_database()
        self.db_manager.create_table()
        self.db_manager.clear_inventory()  # Only call this if you really need to clear the inventory
        self.db_manager.insert_initial_data()
        self.db_manager.initialize_inventory()

    def save_to_json(self, data, filename='gui/manager/data/barcode_data.json'):
        with open(filename, 'w', encoding='utf-8') as json_file:
            json.dump(data, json_file, ensure_ascii=False, indent=4)

    def append_list(self):
        barcodes = []
        try:
            while True:
                barcode_data = input("Scan Barcode (type 'exit' to finish): ")
                if barcode_data.lower() == 'exit':
                    break
                split_data = barcode_data.split(".")
                if len(split_data) != 3:
                    print("Incorrect format. Expected format: A1.04.I1")
                    continue
                item_tag, quantity, inbound_location = split_data
                quantity = int(quantity)  # Ensure quantity is an integer
                barcode_entry = {
                    "time": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    "barcode_id": barcode_data,
                    "inbound_location": inbound_location,
                    "quantity": quantity
                }
                barcodes.append(barcode_entry)
                product_info = self.db_manager.get_product_info(item_tag)
                if product_info:
                    item_id, item_name = product_info
                    inbound_data = {
                        "item_name": item_name,
                        "quantity": quantity,
                        "inbound_zone": inbound_location,
                        "arrival_date": barcode_entry["time"],
                        "current_status": 'waiting'
                    }
                    self.db_manager.save_data("Inbound", inbound_data)
                    self.db_manager.add_to_stock(item_id, quantity)
                else:
                    print(f"Item {item_tag} not found in database.")
                self.save_to_json(barcodes)
                print(f"Barcode {barcode_data} saved.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.db_manager.close_connection()
        print("Scan Completed")

if __name__ == '__main__':
    barcode_scanner = BarcodeScanner()
    barcode_scanner.append_list()
