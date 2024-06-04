import mysql.connector
import pandas as pd

class DatabaseUtils:
    def __init__(self, conn):
        self.conn = conn
        self.cur = conn.cursor()

    def get_product_id(self, product_name):
        query = "SELECT item_id FROM ProductInventory WHERE item_name = %s"
        self.cur.execute(query, (product_name,))
        result = self.cur.fetchone()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        print(f"get_product_id: product_name={product_name}, product_id={result[0] if result else None}")  # 디버깅 정보 출력
        return result[0] if result else None
 
    def get_stock(self, item_id):
        query = "SELECT stock FROM ProductInventory WHERE item_id = %s"
        self.cur.execute(query, (item_id,))
        result = self.cur.fetchone()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return result[0] if result else None

    def update_stock(self, item_id, quantity):
        query = "UPDATE ProductInventory SET stock = stock - %s WHERE item_id = %s"
        self.cur.execute(query, (quantity, item_id))
        self.conn.commit()

    def initialize_inventory(self):
        inventory_data = [
            (1, "Coke", 10),
            (2, "Sprite", 10),
            (3, "Chapagetti", 10),
            (4, "Buldak", 10),
            (5, "Robot Vacuum", 10),
            (6, "Coffee Pot", 10)
        ]
        for item_id, item_name, stock in inventory_data:
            self.cur.execute("INSERT IGNORE INTO ProductInventory (item_id, item_name, stock) VALUES (%s, %s, %s)", (item_id, item_name, stock))
        self.conn.commit()

    def get_last_user_id(self, table_name):
        query = f"SELECT MAX(user_id) FROM {table_name}"
        self.cur.execute(query)
        result = self.cur.fetchone()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return result[0] if result[0] is not None else 0

    def fetch_all_product(self, table_name):
        query = f"SELECT * FROM {table_name}"
        self.cur.execute(query)
        result = self.cur.fetchall()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return result
    
    def fetch_product_orders_dataframe(self):
        product_orders = self.fetch_all_product("ProductOrder")
        df = pd.DataFrame(product_orders, columns=['OrderID', 'UserID', 'ItemID', 'ProductName', 'Quantity', 'OrderTime'])
        return df
