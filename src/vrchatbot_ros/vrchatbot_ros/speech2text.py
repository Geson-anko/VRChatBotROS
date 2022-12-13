import queue

import rclpy
import whisper
from rclpy.node import Node
from std_msgs.msg import String
from vrchatbot.recorder import Recorder
from vrchatbot.speech_recongnition import SpeechRecongition

from .argument_parser import base_parser
from .config_tools import FieldNames as FN
from .config_tools import read_config_file


class Speech2Text(Node):
    """Recognizing Speech and convert to text."""

    def __init__(
        self,
        recorder_cfg: dict,
        speech_recognition_cfg: dict,
        decoding_options: dict,
    ) -> None:
        super().__init__("speech2text")
        logger = self.get_logger()

        logger.info("Setting up...")
        self.publisher_ = self.create_publisher(String, "recognized_text", 10)

        self.speech_recognizer = SpeechRecongition(
            **speech_recognition_cfg,
            options=whisper.DecodingOptions(**decoding_options),
        )
        self.recorder = Recorder(**recorder_cfg)
        self.wave_queue = self.recorder.record_forever_background(is_daemon=True)

        self.timer = self.create_timer(0.001, self.update)

        logger.info("Ready.")

    def update(self):
        logger = self.get_logger()
        try:
            wave = self.wave_queue.get(timeout=5)
            probs, text = self.speech_recognizer.recongnize(wave)
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)

            logger.info(f"lang: {max(probs, key=probs.get)} ,recoginized: {text}")
        except queue.Empty:
            pass


def main(args=None):
    rclpy.init(args=args)

    parser = base_parser()
    arg_local, _ = parser.parse_known_args()
    cfg = read_config_file(arg_local.config_file)

    speech2text = Speech2Text(
        recorder_cfg=cfg[FN.RECORDER],
        speech_recognition_cfg=cfg[FN.SPEECH_RECOGNITION],
        decoding_options=cfg[FN.DECODING_OPTION],
    )

    rclpy.spin(speech2text)

    speech2text.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
