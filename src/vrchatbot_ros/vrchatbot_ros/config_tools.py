from typing import Any

import toml


class FieldNames:
    RECORDER = "Recorder"
    DECODING_OPTION = "DecodingOption"
    SPEECH_RECOGNITION = "SpeechRecognition"
    CHATBOT = "ChatBot"
    TEXT_SPEAKER = "Speaker"
    TEXT_CHAT = "TextChat"


def read_config_file(file_path: Any) -> dict:
    """Read the config file toml."""
    return toml.load(file_path)
