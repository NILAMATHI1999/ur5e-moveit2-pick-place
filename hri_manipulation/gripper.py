class Gripper:
    def __init__(self, logger=None):
        self.logger = logger

    def open(self):
        if self.logger:
            self.logger.info("Gripper OPEN 🟢")
        else:
            print("Gripper OPEN 🟢")

    def close(self):
        if self.logger:
            self.logger.info("Gripper CLOSE 🔴")
        else:
            print("Gripper CLOSE 🔴")
