import sys
import os
import json
from db.src.DatabaseControl import DatabaseManager
from datetime import datetime


class BarcodeScanner:
    def __init__(self):
        db_manager = DatabaseManager(host='localhost')
        db_manager.create_database("amr")
        db_manager.connect_database()
        db_manager.create_table()
        db_manager.save_data("inbound","")
        db_manager.close_connection()


    def save_to_json(data, filename='gui/manager/data/barcode_data.json'):
        with open(filename, 'w', encoding='utf-8') as json_file:
            json.dump(data, json_file, ensure_ascii=False, indent=4)


    def append_list(self):
        barcodes = []

        while True:
            barcode_data = input("Scan Barcode (type 'exit' to finish): ")

            if barcode_data.lower() == 'exit':
                break

            # 여기서 data split하기


            # 바코드 데이터와 타임스탬프를 딕셔너리로 저장
            barcode_entry = {
                "barcode": barcode_data,
                "timestamp": datetime.now().isoformat()
            }
            barcodes.append(barcode_entry)
            
            self.save_to_json(barcodes)
            print(f"Barcode {barcode_data} saved.")

        print("Scan Completed")


if __name__ == '__main__':
        barcode_scanner = BarcodeScanner()
        barcode_scanner.append_list()