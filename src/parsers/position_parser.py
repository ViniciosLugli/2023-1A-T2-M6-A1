from typing import Any, List
from .csv import CSV
from .base import BaseParser
from custom_types import Position

# Class to parse csv data to X and Y coordinates
class PositionCSV(BaseParser):
    def parse(csv_data: List[Any]) -> Any:
        # len(row) == 2 is to ensure that we only get rows with 2 columns (X and Y)
        return [Position(float(row[0]), float(row[1])) for row in csv_data if len(row) == 2]