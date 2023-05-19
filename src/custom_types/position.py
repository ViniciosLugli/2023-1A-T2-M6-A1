class Position:
	def __init__(self, x, y):
		self.__x = x
		self.__y = y

	def __repr__(self):
		return f'Position({self.x}, {self.y})'

	@property
	def x(self):
		return self.__x

	@property
	def y(self):
		return self.__y

	@x.setter
	def x(self, value):
		self.__x = value

	@y.setter
	def y(self, value):
		self.__y = value