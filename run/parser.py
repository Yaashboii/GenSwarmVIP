import argparse
from tabulate import tabulate
import yaml
from ast import literal_eval


class ParameterService:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description="Run simulation with custom parameters.")
        self.args = None

    def add_argument(self, name, **kwargs):
        """
        Add a single argument to the parser.

        :param name: Name of the argument (e.g., '--arg').
        :param kwargs: Additional keyword arguments for `add_argument` method.
        """
        if 'default' in kwargs and 'type' not in kwargs:
            kwargs['type'] = type(kwargs['default'])
        self.parser.add_argument(name, **kwargs)

    def remove_argument(self, name):
        """
        Remove a single argument from the parser.

        :param name: Name of the argument (e.g., '--arg').
        """
        print(f"Removing argument: {name}")
        self.parser._remove_action(name)

    def add_arguments_from_yaml(self, yaml_file):
        """
        Add multiple arguments from a YAML configuration file.

        :param yaml_file: Path to the YAML configuration file.
        """
        with open(yaml_file, 'r') as file:
            print(f"Loading arguments from {yaml_file}")
            config = yaml.safe_load(file)
            arguments = config.get('arguments', {})
            for arg, params in arguments.items():
                if 'default' in params:
                    default_value = params['default']
                    # Safely convert default value to the correct type
                    if isinstance(default_value, str):
                        try:
                            params['default'] = literal_eval(default_value)
                        except (ValueError, SyntaxError):
                            pass  # Keep the default value as a string if it cannot be evaluated
                self.add_argument(arg, **params)
        self.args = self.parse_arguments(self.args)

    def parse_arguments(self, args):
        return self.parser.parse_args(args)

    def get_help(self):
        """
        Get the help message.

        :return: Help message string.
        """
        return self.parser.format_help()

    @staticmethod
    def format_arguments_as_table(args):
        """
        Format parsed arguments as a table.

        :param args: Namespace object containing parsed arguments.
        :return: String representation of arguments in a table format.
        """
        arguments = vars(args)
        table = [[key, type(value).__name__, value] for key, value in arguments.items()]
        return tabulate(table, headers=["Argument", "Type", "Value"], tablefmt="pipe")
