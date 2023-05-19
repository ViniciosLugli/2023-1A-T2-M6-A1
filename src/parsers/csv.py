import os
from .base import BaseParser

class CSV:
    BASEPATH = 'artifacts'

    @staticmethod
    def get_csv_files():
        return [f for f in os.listdir(CSV.BASEPATH) if f.endswith('.csv')]

    def __init__(self, filename: str):
        if CSV.BASEPATH not in filename:
            filename = os.path.join(CSV.BASEPATH, filename)

        if not filename.endswith('.csv'):
            filename += '.csv'

        if not os.path.exists(filename):
            raise Exception(f'File {filename} does not exist')

        self.filename = filename
        self.__raw_data = None
        self.__data = None
        self.__parser = None

    def __read(self):
        with open(self.filename, 'r') as f:
            self.__raw_data = f.read()

    def __parse(self):
        if self.__raw_data is None:
            raise Exception('No data to parse')

        lines = self.__raw_data.split('\n')

        self.__data = [line.split(',') for line in lines]

        if self.__parser is not None:
            self.__data = self.__parser.parse(self.__data)

    def set_parser(self, parser: BaseParser):
        self.__parser = parser


    def get_data(self):
        self.__read()
        self.__parse()
        return self.__data