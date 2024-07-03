def add_arguments(parser, arguments):
    for arg, params in arguments.items():
        parser.add_argument(arg, **params)
