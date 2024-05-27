CREATE TABLE IF NOT EXISTS ProductInventory (
    item_id INT NOT NULL PRIMARY KEY,
    item_name VARCHAR(16) NOT NULL,
    stock INT NOT NULL DEFAULT 8
);

CREATE TABLE IF NOT EXISTS ProductOrder (
    user_id INT NOT NULL,
    item_id INT NOT NULL,
    items VARCHAR(16) NOT NULL,
    quantities INT NOT NULL,
    order_time TIMESTAMP NOT NULL,
    FOREIGN KEY (item_id) REFERENCES ProductInventory(item_id)

);