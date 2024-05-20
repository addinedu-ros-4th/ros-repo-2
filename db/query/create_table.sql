CREATE TABLE IF NOT EXISTS Inbound (
    inbound_id INT AUTO_INCREMENT PRIMARY KEY,
    item_name VARCHAR(40) NOT NULL,
    quantity INT NOT NULL,
    inbound_zone VARCHAR(40) NOT NULL,
    scan_time DATETIME NOT NULL,
    status VARCHAR(40) NOT NULL
);