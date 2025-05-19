    # ---------------------------------------------------------------------------- #
    #                                    Mission                                   #
    # ---------------------------------------------------------------------------- #
    #範例:
    # def templateMission(self):
    #     # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
    #     if self.mode != self.WAIT_MODE:
    #         self.stopMission()
    #         return False
    #     self.__setMode(self.TEMPLATE_MODE)
    #     # --------------------------------- variable --------------------------------- #
    #     # ----------------------------------- 開始任務 ----------------------------------- #
    #     if self.mode != self.TEMPLATE_MODE:
    #         self.node.get_logger().info("It's not in template mode")
    #         self.stopMission()
    #         return False
    #     # ----------------------------------- 結束任務 ----------------------------------- #
    #     self.stopMission()
    #     return True
import math
import rclpy
from rclpy.node import Node
import time
import yaml

class Mission:

    # define mode name
    WAIT_MODE = -1
    NAVIGATE_MODE = 0
    FIRE_MODE = 1
    SPRAY_MODE = 2
    GOHOME_MODE = 3

    # initial mode
    mode = WAIT_MODE
    def __init__(self, node):
        self.node = node
        pass



    def __setMode(self, mode):

        if mode not in [
            self.WAIT_MODE,
            self.NAVIGATE_MODE,
            self.FIRE_MODE,
            self.SPRAY_MODE,
            self.GOHOME_MODE
        ]:
            self.node.get_logger().error("not a valid mode")
            return False
        self.mode = mode

    def stopMission(self):
        self.__setMode(self.WAIT_MODE)
        self.controller.setZeroVelocity()


    def navigateMission(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為導航模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.NAVIGATE_MODE)
        # ----------------------------------- 開始任務 ----------------------------------- #
        if self.mode != self.NAVIGATE_MODE:
            self.node.get_logger().info("It's not in navigate mode")
            self.stopMission()
            return False
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True
    
    def fireMission(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為滅火模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.FIRE_MODE)
        # ----------------------------------- 開始任務 ----------------------------------- #
        if self.mode != self.FIRE_MODE:
            self.node.get_logger().info("It's not in spray mode")
            self.stopMission()
            return False
        
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True

    def sprayMission(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為滅火模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.SPRAY_MODE)
        # ----------------------------------- 開始任務 ----------------------------------- #
        if self.mode != self.SPRAY_MODE:
            self.node.get_logger().info("It's not in spray mode")
            self.stopMission()
            return False
        
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True
    
    def gohomeMission(self):
        # 檢查先前模式是否為等待模式，定且設定目前模式為返航模式
        if self.mode != self.WAIT_MODE:
            self.stopMission()
            return False
        self.__setMode(self.GOHOME_MODE)
        # ----------------------------------- 開始任務 ----------------------------------- #
        if self.mode != self.GOHOME_MODE:
            self.node.get_logger().info("It's not in gohome mode")
            self.stopMission()
            return False
        # ----------------------------------- 結束任務 ----------------------------------- #
        self.stopMission()
        return True
    