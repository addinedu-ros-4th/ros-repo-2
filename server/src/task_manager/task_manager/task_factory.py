import heapq

class Task:
    def __init__(self, task_id, task_type, priority, bundle_id, item, quantity, location, lift):
        self.task_id = task_id
        self.bundle_id = bundle_id
        self.task_type = task_type
        self.priority = priority
        self.item = item
        self.quantity = quantity
        self.location = location
        self.lift = lift


class TaskFactory:
    @staticmethod
    def create_outbound_tasks(bundle_id, tasks):
        transaction_tasks = []
        for task in tasks:
            transaction_tasks.append(task)

        if transaction_tasks:
            initial_task = Task(
                task_id=f'{bundle_id}_initial',
                task_type='delivery_initial',
                priority=min(task.priority for task in transaction_tasks),
                bundle_id=bundle_id,
                item='',
                quantity=0,
                location='O1',
                lift = 'Up'
            )

            final_task = Task(
                task_id=f'{bundle_id}_final',
                task_type='delivery_final',
                priority=min(task.priority for task in transaction_tasks),
                bundle_id=bundle_id,
                item='',
                quantity=0,
                location='O1',
                lift = 'Down'
            )

            transaction_tasks.insert(0, initial_task)
            transaction_tasks.append(final_task)

        return transaction_tasks

    @staticmethod
    def create_inbound_tasks(bundle_id, tasks, initial_location):
        transaction_tasks = []
        for task in tasks:
            transaction_tasks.append(task)

        if transaction_tasks:
            initial_task = Task(
                task_id=f'{bundle_id}_initial',
                task_type='unload_initial',
                priority=min(task.priority for task in transaction_tasks),
                bundle_id=bundle_id,
                item='',
                quantity=0,
                location=initial_location,
                lift = 'Up'
            )

            final_task = Task(
                task_id=f'{bundle_id}_final',
                task_type='unload_final',
                priority=min(task.priority for task in transaction_tasks),
                bundle_id=bundle_id,
                item='',
                quantity=0,
                location='final_location',
                lift = 'Down'
            )

            transaction_tasks.insert(0, initial_task)
            transaction_tasks.append(final_task)

        return transaction_tasks
