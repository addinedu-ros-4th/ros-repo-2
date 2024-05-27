import sys
import os
import json
from db.src.DatabaseManager import DatabaseManager
from datetime import datetime


class BarcodeScanner:
    def __init__(self):
        self.db_manager = DatabaseManager(host='localhost')
        self.db_manager.create_database("amr")
        self.db_manager.connect_database()
        self.db_manager.create_table()

    def save_to_json(self, data, filename='gui/manager/data/barcode_data.json'):
        with open(filename, 'w', encoding='utf-8') as json_file:
            json.dump(data, json_file, ensure_ascii=False, indent=4)

    def append_list(self):
        barcodes = []

        while True:
            # Read Barcode Using Barcode Scanner Machine
            barcode_data = input("Scan Barcode (type 'exit' to finish): ")

            if barcode_data.lower() == 'exit':
                break

            # Split Barcode Data Here
            split_data = barcode_data.split(".")
            if len(split_data) != 3:
                print("Incorrect format. Expected format: A1.04.I1")
                continue

            item_location, quantity, inbound_location = split_data

            # Saving Barcode and Time in Dictionary
            barcode_entry = {
                "time": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                "barcode_id": barcode_data,
                "inbound_location": inbound_location,
                "quantity": quantity
            }

            barcodes.append(barcode_entry)
            
            # Saving Data in Database
            product_info = self.db_manager.get_product_info(item_location)
            if product_info:
                self.db_manager.save_data("inbound", {
                    "inbound_id": None,
                    "inbound_location": inbound_location,
                    "inbound_time": barcode_entry["time"],
                    "quantity": quantity,
                    "barcode_id": barcode_data,
                })

            else:
                print(f"Item Location {item_location} not found in database.")

            self.save_to_json(barcodes)
            print(f"Barcode {barcode_data} saved.")

        print("Scan Completed")


if __name__ == '__main__':
    barcode_scanner = BarcodeScanner()
    barcode_scanner.append_list()
