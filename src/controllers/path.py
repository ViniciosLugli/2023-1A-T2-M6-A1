from collections import deque
from custom_types import PoseData, Position
from parsers import CSV, PositionCSV

class PathController(deque):
    def __init__(self, csv_name: str = None):
        super().__init__()
        if csv_name is None:
            return

        csv = CSV(csv_name)
        csv.set_parser(PositionCSV)

        for position in csv.get_data():
            self.enqueue(position)

    def enqueue(self, x):
        super().append(x)

    def dequeue(self):
        return super().popleft()

    def get_next_position(self) -> Position:
        return self.dequeue()

    def current_position(self) -> Position:
        return self[0]

    def get_inverse_path(self) -> 'PathController':
        inverse_path = PathController()
        for position in self:
            inverse_path.enqueue(Position(-position.x, -position.y))
        return inverse_path