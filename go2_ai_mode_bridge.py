#!/usr/bin/env python3
import sys
import time

# Prefer the full SDK2 source tree over a potentially incomplete ROS overlay install.
SDK2_SRC = "/home/taehyun/go2_ws/src/unitree_sdk2_python"
if SDK2_SRC not in sys.path:
    sys.path.insert(0, SDK2_SRC)

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

class Go2AiModeBridge(Node):
    def __init__(self):
        super().__init__("go2_ai_mode_bridge")

        self.ms = MotionSwitcherClient()
        self.ms.SetTimeout(5.0)
        self.ms.Init()

        self.oa = ObstaclesAvoidClient()
        self.oa.SetTimeout(5.0)
        self.oa.Init()

        self.rs = RobotStateClient()
        self.rs.SetTimeout(5.0)
        self.rs.Init()

        self.srv = self.create_service(SetBool, "/go2/set_ai_mode", self.cb)
        self.get_logger().info("service ready: /go2/set_ai_mode")

    def _service_status_map(self):
        code, services = self.rs.ServiceList()
        if code != 0 or services is None:
            return code, None
        mapped = {s.name: int(s.status) for s in services}
        return 0, mapped

    def _service_status(self):
        code, mapped = self._service_status_map()
        if code != 0 or mapped is None:
            return f"ServiceList:{code}"
        names = ("sport_mode", "ai_sport", "advanced_sport", "motion_switcher", "obstacles_avoid")
        return ", ".join(f"{n}={mapped.get(n, -1)}" for n in names)

    def _ensure_service_enabled(self, name: str):
        def poll_enabled():
            for _ in range(10):
                time.sleep(0.2)
                c, m = self._service_status_map()
                if c == 0 and m is not None and m.get(name, -1) == 1:
                    return True
            return False

        code, mapped = self._service_status_map()
        if code != 0 or mapped is None:
            return code, f"ServiceList failed: {code}"

        if name not in mapped:
            return 1, f"service not found: {name}"

        if mapped[name] == 1:
            return 0, f"{name} already enabled"

        switch_code = self.rs.ServiceSwitch(name, True)
        if switch_code != 0:
            return switch_code, f"ServiceSwitch({name}, True) failed: {switch_code}"

        # Some firmwares return call_code=0 but keep status at 0, so verify with polling.
        if poll_enabled():
            return 0, f"{name} enabled"

        # Retry once with toggle sequence.
        self.rs.ServiceSwitch(name, False)
        time.sleep(0.2)
        retry_code = self.rs.ServiceSwitch(name, True)
        if retry_code == 0 and poll_enabled():
            return 0, f"{name} enabled after retry toggle"

        code, mapped = self._service_status_map()
        after = -1 if mapped is None else mapped.get(name, -1)
        return 2, (
            f"{name} remains disabled after ServiceSwitch "
            f"(first_call={switch_code}, retry_call={retry_code}, status={after})"
        )

    def cb(self, req, res):
        if req.data:
            # Keep current mode so we can restore it if ai selection fails.
            check_code, current = self.ms.CheckMode()
            current_name = ""
            if check_code == 0 and current is not None:
                current_name = str(current.get("name", ""))

            if current_name == "mcf":
                # On some versions ai mode selection is rejected while mcf is current.
                self.ms.SelectMode("normal")
                time.sleep(0.3)

            for service_name in ("motion_switcher", "obstacles_avoid"):
                code, detail = self._ensure_service_enabled(service_name)
                if code != 0:
                    res.success = False
                    res.message = f"{detail} [{self._service_status()}]"
                    return res

            mode_errors = []
            selected_name = None
            for name in ("ai", "ai_sport"):
                code, _ = self.ms.SelectMode(name)
                if code == 0:
                    selected_name = name
                    break
                mode_errors.append(f"{name}:{code}")

            if selected_name is None:
                # Fail-safe: try to restore previous mode instead of leaving robot limp.
                if current_name:
                    self.ms.SelectMode(current_name)
                else:
                    self.ms.SelectMode("normal")
                res.success = False
                mode_dbg = f"check_code={check_code}, current='{current_name}'"
                svc_dbg = self._service_status()
                res.message = "SelectMode failed (" + ", ".join(mode_errors) + f") [{mode_dbg}] [{svc_dbg}]"
                return res

            code = self.oa.SwitchSet(True)
            if code != 0:
                res.success = False
                res.message = f"SwitchSet(True) failed: {code}"
                return res
            g_code, enabled = self.oa.SwitchGet()
            res.success = True
            res.message = f"AI ON ({selected_name}), avoid={enabled}, get_code={g_code}"
            return res

        # Try to return to normal mode; avoid ReleaseMode() to prevent losing posture.
        code, _ = self.ms.SelectMode("normal")
        if code != 0:
            res.success = False
            res.message = f"SelectMode(normal) failed: {code} [{self._service_status()}]"
            return res
        code = self.oa.SwitchSet(False)
        if code != 0:
            res.success = False
            res.message = f"SwitchSet(False) failed: {code}"
            return res
        g_code, enabled = self.oa.SwitchGet()
        res.success = True
        res.message = f"AI OFF, avoid={enabled}, get_code={g_code}"
        return res

def main():
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])  # ex: enp2s0
    else:
        ChannelFactoryInitialize(0)

    rclpy.init()
    node = Go2AiModeBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
