import mysql.connector
import os
import configparser
from DatabaseUtils import DatabaseUtils

class DatabaseManager:
    def __init__(self, host):
        self.host = host
        self.user = "root"
        self.db_name = "amr"
        self.cur = None
        self.conn = None
        self.password = self.get_password_from_config()
        self.utils = None 

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
        self.utils = DatabaseUtils(self.conn) 
    
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

    # 얘는...모르겠음
    def insert_initial_data(self):
        # Check if ProductInfo table is empty before inserting initial data
        self.cur.execute("SELECT COUNT(*) FROM ProductInfo")
        result = self.cur.fetchone()
        if result[0] > 0:
            print("Initial data already exists in ProductInfo table.")
            return
        
        initial_data = [
            ("A1.04.I1", "Coke", "A1", 1.25, "Drink"),
            ("A2.04.I1", "Sprite", "A2", 1.25, "Drink"),
            ("B1.04.I2", "Chapagetti", "B1", 0.5, "Food"),
            ("B2.04.I2", "Buldak", "B2", 0.5, "Food"),
            ("C1.04.I3", "Robot Vacuum", "C1", 3.0, "Appliance"),
            ("C2.04.I3", "Coffee Pot", "C2", 1.0, "Appliance")
        ]

        for item_id, item_name, item_tag, item_weight, item_category in initial_data:
            self.cur.execute("INSERT IGNORE INTO ProductInfo (barcode_id, item_name, item_tag, item_weight, item_category) VALUES (%s, %s, %s, %s, %s)", (item_id, item_name, item_tag, item_weight, item_category))
        self.conn.commit()

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
        """
        테이블에 데이터를 저장하는 함수
        :param table_name: 테이블 이름
        :param data: 컬럼 이름을 키로 하고 값을 값으로 하는 딕셔너리
        :return: 삽입된 데이터의 ID (기본 키가 AUTO_INCREMENT 인 경우)
        """
        columns = ', '.join(data.keys())
        placeholders = ', '.join(['%s' for _ in data])
        query = f"INSERT INTO {table_name} ({columns}) VALUES ({placeholders})"
        
        self.cur.execute(query, tuple(data.values()))
        self.conn.commit()

        self.cur.execute("SELECT LAST_INSERT_ID()")
        last_id = self.cur.fetchone()[0]
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return last_id
    
    def get_data(self, table_name, columns=None, condition=None):
        """
        테이블에서 데이터를 읽어오는 함수
        :param table_name: 테이블 이름
        :param columns: 선택적으로 읽어올 컬럼 리스트 (기본값: 모든 컬럼)
        :param condition: 선택적으로 데이터를 필터링할 조건 (기본값: 없음)
        :return: 조건에 맞는 데이터의 리스트
        """
        if columns:
            columns_str = ', '.join(columns)
        else:
            columns_str = '*'
        
        query = f"SELECT {columns_str} FROM {table_name}"
        if condition:
            query += f" WHERE {condition}"
        
        self.cur.execute(query)
        rows = self.cur.fetchall()
        return rows

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

    def remove_from_stock(self, item_id, quantity):
        query = "UPDATE ProductInventory SET stock = stock - %s WHERE item_id = %s"
        self.cur.execute(query, (quantity, item_id))
        self.conn.commit()
    # 얘도 
    def add_to_stock(self, item_id, quantity):
        query = "UPDATE ProductInventory SET stock = stock + %s WHERE item_id = %s"
        self.cur.execute(query, (quantity, item_id))
        self.conn.commit()

    # 내가 추가한거 다른데로 이동
    def initialize_inventory(self):
        inventory_data = [
            (1, "Coke", 0),
            (2, "Sprite", 0),
            (3, "Chapagetti", 0),
            (4, "Buldak", 0),
            (5, "Robot Vacuum", 0),
            (6, "Coffee Pot", 0)
        ]
        for item_id, item_name, stock in inventory_data:
            self.cur.execute("INSERT IGNORE INTO ProductInventory (item_id, item_name, stock) VALUES (%s, %s, %s)", (item_id, item_name, stock))
            # print(f"Inserted {item_name} with item_id={item_id} and stock={stock}")  # 디버깅 정보 출력
        self.conn.commit()
    # 이것도 마찬가지
    def clear_inventory(self):
        query = "DELETE FROM ProductInventory"
        self.cur.execute(query)
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
        self.close_connection()
        return result


    def get_last_user_id(self):
        query = "SELECT MAX(user_id) FROM ProductOrder"
        self.cur.execute(query)
        result = self.cur.fetchone()
        self.cur.fetchall()  # 이전 쿼리의 결과를 모두 읽음
        return result[0] if result[0] is not None else 0

    

    # Update table info
    def fetch_all_product(self, table_name):
        query = f"SELECT * FROM {table_name}"
        self.cur.execute(query)
        result = self.cur.fetchall()
        self.cur.fetchall() 
        return result
    
if __name__ == '__main__':
    db_manager = DatabaseManager(host='localhost')
    
    db_manager.connect_database()
    db_manager.create_table()
    db_manager.insert_initial_data()
    db_manager.initialize_inventory()
    db_manager.close_connection()