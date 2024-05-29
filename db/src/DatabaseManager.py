import mysql.connector
import os
import configparser

class DatabaseManager:
    def __init__(self, host):
        self.host = host
        self.user = "root"
        self.db_name = "amr"
        self.cur = None
        self.conn = None
        self.password = self.get_password_from_config()


    def get_password_from_config(self):
        config = configparser.ConfigParser()
        config.read('db/config/config.ini')
        return config['database']['password']
    
    
    def connect_database(self, db_name=None):
        if db_name is None:
            db_name = self.db_name
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                database=db_name,
                password=self.password
            )
        except mysql.connector.Error as err:
            if err.errno == mysql.connector.errorcode.ER_BAD_DB_ERROR:
                self.conn = mysql.connector.connect(
                    host=self.host,
                    user=self.user,
                    password=self.password
                )
                self.cur = self.conn.cursor()
                self.create_database(db_name)
                self.conn.database = db_name
            else:
                raise
        self.cur = self.conn.cursor()
    
    def create_database(self, db_name):
        try:
            self.cur.execute(f"CREATE DATABASE {db_name}")
        except mysql.connector.Error as err:
            print(f"Failed creating database: {err}")
            exit(1)
        print(f"Database {db_name} created successfully.")
        self.cur.execute(f"USE {db_name}")
    
    def create_table(self):
        self.execute_sql_file("db/query/create_table.sql")
        self.initialize_inventory()
    
    def execute_sql_file(self, file_path):
        with open(file_path, 'r') as file:
            sql_script = file.read()
        commands = sql_script.split(';')
        for command in commands:
            try:
                if command.strip() != '':
                    self.cur.execute(command)
            except mysql.connector.Error as err:
                print(f"Error occurred: {err}")
        self.conn.commit()
    
    def save_data(self, table_name, data):
        columns = ', '.join(data.keys())
        placeholders = ', '.join(['%s' for _ in data])
        query = f"INSERT INTO {table_name} ({columns}) VALUES ({placeholders})"
        self.cur.execute(query, tuple(data.values()))
        self.conn.commit()
        self.cur.execute("SELECT LAST_INSERT_ID()")
        last_id = self.cur.fetchone()[0]
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return last_id

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
            (1, "cola", 10),
            (2, "cider", 10),
            (3, "coffee", 10),
            (4, "water", 10),
            (5, "Jin_ramen", 10),
            (6, "Chapagetti", 10),
            (7, "Bibimmyeon", 10),
            (8, "robot_cleaner", 10),
            (9, "radio", 10), 
            (10, "tv", 10)
        ]
        for item_id, item_name, stock in inventory_data:
            self.cur.execute("INSERT IGNORE INTO ProductInventory (item_id, item_name, stock) VALUES (%s, %s, %s)", (item_id, item_name, stock))
            # print(f"Inserted {item_name} with item_id={item_id} and stock={stock}")  # 디버깅 정보 출력
        self.conn.commit()


    def close_connection(self):
        if self.cur:
            self.cur.close()
        if self.conn:
            self.conn.close()
    
    def find_elements(self, name, password):
        query = "SELECT UserId, Name, Password from Users where Name = %s and (Password) = %s;"
        self.cur.execute(query, (name, password))
        result = self.cur.fetchone()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        self.close_connection()
        return result

    def get_last_user_id(self):
        query = "SELECT MAX(user_id) FROM ProductOrder"
        self.cur.execute(query)
        result = self.cur.fetchone()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return result[0] if result[0] is not None else 0

    def fetch_all_product_orders(self):
        query = "SELECT * FROM ProductOrder"
        self.cur.execute(query)
        result = self.cur.fetchall()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return result
    
if __name__ == '__main__':
    db_manager = DatabaseManager(host='localhost')
    db_manager.connect_database()
    db_manager.create_table()
    db_manager.close_connection()