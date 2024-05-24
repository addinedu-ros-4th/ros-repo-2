import mysql.connector
import pandas as pd

class DatabaseManager:
    def __init__(self, host):
        self.host = host
        self.user = "root"
        self.db_name = "amr"
        self.cur = None
        self.conn = None
        self.password = "1234"
    
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
    
    
    # Save in Database 'amr'
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
        
        return last_id
        

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
    
    
if __name__ == '__main__':
    db_manager = DatabaseManager(host='localhost')

    inbound_data = {
        'item_name': 'Cola',
        'quantity': 4,
        'inbound_zone': 'I1',
        'scan_time': '2024-05-20 12:16:57',
        'status': 'wait'
    }
    
    db_manager.connect_database()
    db_manager.create_table()
    last_id = db_manager.save_data('Inbound', inbound_data)
    print(f"Inbound order added with ID: {last_id}")
    
    db_manager.close_connection()