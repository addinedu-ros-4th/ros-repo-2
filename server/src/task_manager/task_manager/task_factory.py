import heapq

class Task:
    def __init__(self, task_id, task_type, priority, bundle_id, item, quantity, location):
        self.task_id = task_id
        self.bundle_id = bundle_id
        self.task_type = task_type
        self.priority = priority
        self.item = item
        self.quantity = quantity
        self.location = location


class TaskFactory:
    @staticmethod
    def create_task(task_id, task_type, priority, bundle_id, item, quantity, location):
        # Create a task with given parameters
        return Task(
            task_id=task_id,
            task_type=task_type,
            priority=priority,
            bundle_id=bundle_id,
            item=item,
            quantity=quantity,
            location=location
        )


    @staticmethod
    def add_initial_task(tasks, initial_location):
        # Add an initial task to a transaction
        initial_task = TaskFactory.create_task(
            task_id=f'{tasks[0].task_id}_initial',
            task_type=tasks[0].task_type,
            priority=tasks[0].priority,
            bundle_id=tasks[0].bundle_id,
            item=tasks[0].item,
            quantity=tasks[0].quantity,
            location=initial_location
        )
        return [initial_task] + tasks


    @staticmethod
    def add_final_task(tasks, final_location):
        # Add a final task to a transaction
        final_task = TaskFactory.create_task(
            task_id = f'{tasks[-1].task_id}_final',
            task_type = tasks[-1].task_type,
            priority = tasks[-1].priority,
            bundle_id = tasks[-1].bundle_id,
            item = tasks[-1].item,
            quantity = tasks[-1].quantity,
            location = final_location
        )
        return tasks + [final_task]


    @staticmethod
    def create_inbound_tasks(self, tasks):
        # Create tasks for an inbound transaction
        storage_task = TaskFactory.create_task(
            task_id=f'{tasks.task_id}_storage',
            task_type='inbound',
            priority=tasks.priority,
            bundle_id=tasks.bundle_id,
            item=tasks.item,
            quantity=tasks.quantity,
            location=tasks.location
        )
        return TaskFactory.add_initial_task([storage_task], initial_location)

    
    @staticmethod
    def create_outbound_tasks(task):
        # Create tasks for an outbound transaction
        delivery_task = TaskFactory.create_task(
            task_id=f'{task.task_id}_delivery',
            task_type='OB',
            priority=tasks[0].priority,
            bundle_id=tasks[0].bundle_id,
            item=task.item,
            quantity=task.quantity,
            location=task.location
        )
        tasks = [delivery_task]
        tasks = TaskFactory.add_initial_task(tasks, 'O1')
        tasks = TaskFactory.add_final_task(tasks, 'O1')
        return tasks
    
    
    # Add task location in transaction
    def add_collection_tasks(self, tasks):
        initial_task = Task(
            task_id=f'{tasks[0].task_id}_initial',
            task_type='OB',
            priority=tasks[0].priority,
            bundle_id=tasks[0].bundle_id,
            item='',
            quantity=0,
            location='O1'
        )
        final_task = Task(
            task_id=f'{tasks[0].task_id}_final',
            task_type='OB',
            priority=tasks[0].priority,
            bundle_id=tasks[0].bundle_id,
            item='',
            quantity=0,
            location='O1'
        )
        return [initial_task] + tasks + [final_task]