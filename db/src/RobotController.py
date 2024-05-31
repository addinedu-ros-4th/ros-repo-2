from db.src.DatabaseManager import DatabaseManager

class RobotController(DatabaseManager):
    def __init__(self, host):
        super().__init__(host)
        self.connect_database()
        self.create_robot_status_table()

    def create_robot_status_table(self):
        create_table_query = """
        CREATE TABLE IF NOT EXISTS RobotStatus (
            RobotID VARCHAR(255) PRIMARY KEY,
            Status VARCHAR(255)
        )
        """
        self.cur.execute(create_table_query)
        self.conn.commit()

    def get_robot_status(self, robot_id):
        query = "SELECT Status FROM RobotStatus WHERE RobotID = %s"
        self.cur.execute(query, (robot_id,))
        result = self.cur.fetchone()
        if result:
            return result[0]
        else:
            return 'IDLE'  # 기본 상태를 IDLE로 설정

    def update_robot_status(self, robot_id, status):
        query = """
        INSERT INTO RobotStatus (RobotID, Status)
        VALUES (%s, %s)
        ON DUPLICATE KEY UPDATE Status = VALUES(Status)
        """
        self.cur.execute(query, (robot_id, status))
        self.conn.commit()
