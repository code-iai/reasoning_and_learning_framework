import re

regex = r"(.+?)\((.+)\)"


class Query:
    def __init__(self, id, mode, query_text):
        self.id = id
        self._mode = mode
        self._query_text =  query_text
        self._predicate, self._parameters = self._parse_query()

    def _parse_query(self):
        matches = re.finditer(regex, self._query_text)

        for match in matches:
            return match.group(1), match.group(2).split(',')
