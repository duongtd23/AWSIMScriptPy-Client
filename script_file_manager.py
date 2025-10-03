from pathlib import Path
import argparse
from core.client_ros_node import *

class AWSIMScriptClient:
    def __init__(self, node, dir_path, wait_writing_trace):
        """
        :param node:
        :param dir_path: must be already validated it existence
        :param wait_writing_trace:
        """
        self.node = node
        self.dir_path = Path(dir_path)
        self.wait_writing_trace = wait_writing_trace
        self.ready_for_new_script = False

    def execute(self):
        script_files = sorted(
            [f for f in self.dir_path.glob("*.script") if f.is_file()],
            key=lambda f: f.name
        )
        for script_file in script_files:
            self.node.send_request(str(script_file))
            time.sleep(15)
            self.loop_wait()
            self.reset()
            time.sleep(3)

    def loop_wait(self):
        while not self.ready_for_new_script:
            self.node.upd_execution_state()
            if self.node.ads_internal_status == AdsInternalStatus.GOAL_ARRIVED:
                self.node.publish_finish_signal()
                if not self.wait_writing_trace:
                    break
                else:
                    res = self.node.query_recording_state()
                    if (res.state is not MonitorRecordingState.Response.WRITING_DATA and
                            res.state is not MonitorRecordingState.Response.RECORDING):
                        break
            print("[INFO] Waiting Ego arrive goal.")
            time.sleep(2)

    def reset(self):
        self.ready_for_new_script = False
        self.node.clear_route()
        self.node.remove_npcs()
        self.node.ads_internal_status = AdsInternalStatus.UNINITIALIZED
        self.node.ego_motion_state = MOTION_STATE_STOPPED
        self.node.published_finish_signal = False

def loop_wait(node):
    while True:
        node.upd_execution_state()
        if node.ads_internal_status == AdsInternalStatus.GOAL_ARRIVED:
            node.publish_finish_signal()
            break
        print("[INFO] Waiting Ego arrive goal.")
        time.sleep(5)

    node.clear_route()
    node.remove_npcs()


def parse_args():
    parser = argparse.ArgumentParser(description='AWSIM-Script client.')
    parser.add_argument(
        'file_or_dir',
        help='Path to a single script file or directory where script files are located.'
    )
    parser.add_argument('-w', '--wait_writing_trace',
                        help='true or false (default: true); To wait for writing trace before executing the next script.')
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    rclpy.init()
    node = ClientNode()

    full_path = os.path.abspath(args.file_or_dir)
    to_wait_writing_trace = True
    if args.wait_writing_trace and args.wait_writing_trace.upper() == 'FALSE':
        to_wait_writing_trace = False

    if os.path.isdir(full_path):
        AWSIMScriptClient(node, full_path, to_wait_writing_trace).execute()
    elif os.path.isfile(full_path):
        node.send_request(full_path)
        time.sleep(15)
        loop_wait(node)
    else:
        print('[ERROR] File or directory not found.')

    node.destroy_node()
    rclpy.shutdown()