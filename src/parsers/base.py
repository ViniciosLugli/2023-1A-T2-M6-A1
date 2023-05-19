from abc import ABC, abstractmethod
from typing import Any, List

class BaseParser(ABC):
	@abstractmethod
	def parse(self, csv_data: List[Any]) -> Any:
		pass