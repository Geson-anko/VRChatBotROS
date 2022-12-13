from argparse import ArgumentParser
from typing import Optional


def base_parser(parser: Optional[ArgumentParser] = None) -> ArgumentParser:
    if parser is None:
        parser = ArgumentParser()

    parser.add_argument(
        "-c", "--config_file", type=str, help="configuration file path.", default="."
    )
    return parser


def chatbot_parser(parser: Optional[ArgumentParser] = None) -> ArgumentParser:
    if parser is None:
        parser = ArgumentParser()

    parser.add_argument(
        "--responce_rate",
        type=float,
        help="Responce rate for text chat [seconds]",
        default=3.0,
    )
    return parser
