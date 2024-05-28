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