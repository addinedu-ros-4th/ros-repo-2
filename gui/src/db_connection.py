# db_connection.py
import mysql.connector

def create_db_connection():
    connection = mysql.connector.connect(
        host='localhost', # MySQL 서버 호스트 이름 또는 IP 주소
        user='root',      # MySQL 사용자 이름
        password='1234',  # MySQL 사용자 비밀번호
        database='amr'    # 연결할 데이터베이스 이름
    )
    return connection
