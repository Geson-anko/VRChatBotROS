import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vrchatbot.text_speaker import TextSpeaker

from .argument_parser import base_parser
from .config_tools import FieldNames as FN
from .config_tools import read_config_file


class Text2Speech(Node):
    """Convert text to human voice.

    Speak the last element of Queue.
    """

    def __init__(self, text_speaker_cfg: dict) -> None:
        super().__init__("text2speech")

        logger = self.get_logger()

        logger.info("Setting up...")

        self.text_speaker = TextSpeaker(**text_speaker_cfg)

        self.responce_text_queue = queue.Queue()
        self.responce_text_subscriber = self.create_subscription(
            String, "responce_text", self.receive_responce_text, 1
        )

        self.timer = self.create_timer(0.01, self.update)

        logger.info("Ready.")

    def receive_responce_text(self, msg: String) -> None:
        """Receiving responce text and put to queue."""
        self.responce_text_queue.put(str(msg.data))

    def update(self):
        logger = self.get_logger()
        try:
            if self.responce_text_queue.qsize() == 0:
                responce_text = self.responce_text_queue.get(timeout=5)
            else:
                for _ in range(self.responce_text_queue.qsize()):
                    responce_text = self.responce_text_queue.get(timeout=0.1)
            logger.info(f"Speak: {responce_text}")
            self.text_speaker.speak_text(responce_text)

            logger.info(f"Spoken: {responce_text}")

        except queue.Empty:
            pass


def main(args=None):
    rclpy.init(args=args)

    parser = base_parser()
    arg_local, _ = parser.parse_known_args()
    cfg = read_config_file(arg_local.config_file)

    text2speech = Text2Speech(cfg[FN.TEXT_SPEAKER])
    rclpy.spin(text2speech)

    text2speech.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
