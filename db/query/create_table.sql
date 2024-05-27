CREATE TABLE IF NOT EXISTS ProductInventory (
    item_id INT NOT NULL PRIMARY KEY,
    item_name VARCHAR(16) NOT NULL,
    stock INT NOT NULL DEFAULT 8
);

CREATE TABLE IF NOT EXISTS ProductOrder (
    order_id INT NOT NULL AUTO_INCREMENT PRIMARY KEY,
    user_id INT NOT NULL,
    item_id INT NOT NULL,
    quantities INT NOT NULL,
    order_time TIMESTAMP NOT NULL,
    FOREIGN KEY (item_id) REFERENCES ProductInventory(item_id)
);

CREATE TABLE IF NOT EXISTS ProductInfo (
    item_id INT AUTO_INCREMENT PRIMARY KEY,
    barcode_id TEXT,
    item_name VARCHAR(16) NOT NUlL,
    item_location VARCHAR(100) NOT NULL,
    item_weight FLOAT NOT NULL,
    category VARCHAR(255)
);

CREATE TABLE IF NOT EXISTS Inbound ( 
    inbound_id INT AUTO_INCREMENT PRIMARY KEY, 
    supplier_id INT NOT NULL, 
    arrival_date TIMESTAMP DEFAULT CURRENT_TIMESTAMP, 
); 