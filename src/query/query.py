import re

regex = r"(.+?)\((.+)\)"


class Query:
    def __init__(self, id, mode, query_text):
        self.id = id
        self._mode = mode
        self._query_text =  query_text
        self.predicate, self._parameters = self._parse_query()

    def _parse_query(self):
        matches = re.finditer(regex, self._query_text)

        for match in matches:
            return match.group(1), _parse_parameters(match.group(2))


def _parse_parameters(matched_parameter_group):
    parameters = []
    brackets_stack = 0
    list_parameter = []

    for parameter in matched_parameter_group.split(','):
        brackets_stack += parameter.count('[')
        brackets_stack -= parameter.count(']')

        if brackets_stack == 0:
            list_parameter.append(parameter)

            parameters.append(','.join(list_parameter))

            list_parameter = []
        else:
            list_parameter.append(parameter)

    return parameters