CREATE TABLE IF NOT EXISTS ProductOrder (
    user_id INT NOT NULL,
    item_id INT NOT NULL,
    items VARCHAR(16) NOT NULL,
    quantities INT NOT NULL,
    order_time TIMESTAMP NOT NULL

);
