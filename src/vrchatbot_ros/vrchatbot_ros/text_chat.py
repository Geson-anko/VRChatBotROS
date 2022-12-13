import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vrchatbot.chatbot import ChatBot

from .argument_parser import base_parser
from .config_tools import FieldNames as FN
from .config_tools import read_config_file


class TextChat(Node):
    """Text chatting with OpenAI GPT."""

    def __init__(self, chatbot_cfg: dict, responce_rate: float = 3.0) -> None:
        super().__init__("text_chat")
        logger = self.get_logger()

        logger.info("Setting up...")
        self.responce_publisher = self.create_publisher(String, "responce_text", 1)

        self.chatbot = ChatBot(**chatbot_cfg)
        self.recognized_text_queue = queue.Queue()
        self.recognized_text_subscriber = self.create_subscription(
            String, "recognized_text", self.receive_recognized_text, 10
        )

        self.responce_rate = responce_rate
        self.timer = self.create_timer(responce_rate, self.update)

        logger.info("Ready.")

    def receive_recognized_text(self, msg: String) -> None:
        """Receiving recognized text and put to queue."""
        self.recognized_text_queue.put(str(msg.data))

    def update(self):
        logger = self.get_logger()
        try:
            if self.recognized_text_queue.qsize() == 0:
                return
            else:
                q_size = self.recognized_text_queue.qsize()
                user_inputs = [
                    self.recognized_text_queue.get(timeout=0.1) for _ in range(q_size)
                ]
                user_input = " ".join(user_inputs)
            responce = self.chatbot.responce(user_input)
            logger.info(f"User input: {user_input}")
            msg = String()
            msg.data = responce
            self.responce_publisher.publish(msg)

            logger.info(f"Responce: {responce}")
        except queue.Empty:
            pass
        except Exception as e:
            logger.fatal(e)


def main(args=None):
    rclpy.init(args=args)

    parser = base_parser()
    arg_local, _ = parser.parse_known_args()
    cfg = read_config_file(arg_local.config_file)

    text_chat = TextChat(chatbot_cfg=cfg[FN.CHATBOT], **cfg[FN.TEXT_CHAT])

    rclpy.spin(text_chat)

    text_chat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
