class LocationAllocator:
    def __init__(self, db_manager):
        self.db_manager = db_manager

    def get_initial_and_final_location(self, item_name):
        # Get the initial location from the Inbound table based on the item name
        query = "SELECT inbound_zone FROM Inbound WHERE item_name = %s ORDER BY arrival_date DESC LIMIT 1"
        self.db_manager.cur.execute(query, (item_name,))
        result = self.db_manager.cur.fetchone()
        
        if not result:
            raise ValueError(f"No initial location found for item {item_name}.")

        initial_location = result[0]

        # Get the item_id based on the task item
        item_id = self.db_manager.get_product_id(item_name)
        
        if item_id is None:
            raise ValueError(f"Item {item_name} not found in database.")
        
        # Get the current stock of the item
        stock = self.db_manager.get_stock(item_id)
        
        if stock is None:
            raise ValueError(f"Failed to retrieve stock for item_id {item_id}.")
        
        # Determine the final location based on stock quantity
        base_location = 'A1'  # Example base location, this could be dynamic based on task details
        
        if stock >= 8:
            final_location = 'Overstocked'
        elif stock >= 4:
            final_location = f"{base_location}-2"
        else:
            final_location = f"{base_location}-1"
        
        return initial_location, final_location
