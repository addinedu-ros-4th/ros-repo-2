import mysql.connector

class DatabaseManager:
    def __init__(self, host):
        self.host = host
        self.user = "root"
        self.db_name = "amr"
        self.cur = None
        self.conn = None
        self.password = "0000"
    
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
        self.execute_sql_file("/home/addinedu/testdb/db/query/create_table.sql")  # 올바른 경로로 수정
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
        product_ids = {"cola": 1, "water": 2, "ramen": 3}
        return product_ids.get(product_name.lower(), None)

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
            (1, "cola", 100),
            (2, "water", 100),
            (3, "ramen", 100)
        ]
        for item_id, item_name, stock in inventory_data:
            self.cur.execute("INSERT IGNORE INTO ProductInventory (item_id, items, stock) VALUES (%s, %s, %s)", (item_id, item_name, stock))
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

if __name__ == '__main__':
    db_manager = DatabaseManager(host='localhost')
    db_manager.connect_database()
    db_manager.create_table()
    db_manager.close_connection()
