CREATE TABLE IF NOT EXISTS ProductInventory (
    item_id INT NOT NULL PRIMARY KEY,
    item_name VARCHAR(16) NOT NULL,
    stock INT NOT NULL DEFAULT 10
);

CREATE TABLE IF NOT EXISTS ProductOrder (
    order_id INT AUTO_INCREMENT PRIMARY KEY,
    user_id INT NOT NULL,
    item_id INT NOT NULL,
    item_name VARCHAR(16) NOT NULL,
    quantities INT NOT NULL,
    order_time TIMESTAMP NOT NULL,
    FOREIGN KEY (item_id) REFERENCES ProductInventory(item_id)
);

CREATE TABLE IF NOT EXISTS ProductInfo (
    item_id INT AUTO_INCREMENT PRIMARY KEY,
    barcode_id TEXT,
    item_name VARCHAR(16) NOT NUlL,
    item_tag VARCHAR(100) NOT NULL,
    item_weight FLOAT NOT NULL,
    item_category VARCHAR(255) NOT NULL
);

CREATE TABLE IF NOT EXISTS Inbound ( 
    inbound_id INT AUTO_INCREMENT PRIMARY KEY, 
    item_name VARCHAR(16) NOT NUlL,
    quantity INT NOT NULL,
    inbound_zone VARCHAR(16) NOT NULL,
    arrival_date TIMESTAMP DEFAULT CURRENT_TIMESTAMP, 
    current_status VARCHAR(255)
); 


CREATE TABLE IF NOT EXISTS TaskList (
    task_id INT AUTO_INCREMENT PRIMARY KEY,
    task_type VARCHAR(36) NOT NULL,
    task_name VARCHAR(36) NOT NULL,
    allocate BOOLEAN,
    robot_id VARCHAR(36)
    perform BOOLEAN,
);


CREATE TABLE IF NOT EXISTS RobotStatus (
    RobotID VARCHAR(255) PRIMARY KEY,
    Status VARCHAR(255)
);