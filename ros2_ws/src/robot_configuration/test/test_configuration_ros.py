"""Rzeczywisty ROS/RSP i pliki, atrapiony właściciel sterowania, bez CAN i ruchu."""
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from robot_control_interfaces.srv import ConfigureRobot, ConfigurationCommand
from robot_control_interfaces.msg import MotionSettings
from robot_configuration.node import ConfigurationNode
from robot_configuration.schema import empty_document, build_urdf


def test_configuration_rpc_updates_real_description_si_profiles_files_and_tf(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '130'); monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', '/tmp/robot-configuration-ros-test-logs')
    model = tmp_path/'rsp.yaml'
    model.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': build_urdf(empty_document(),'initial'), 'frame_prefix': 'preview/'}}}))
    executable = Path(get_package_prefix('robot_state_publisher'))/'lib/robot_state_publisher/robot_state_publisher'
    process = subprocess.Popen([str(executable), '--ros-args', '-r', '__ns:=/robot/preview', '-r', '__node:=state_publisher', '--params-file', str(model),
        '-r', '/tf:=/robot/preview/tf', '-r', '/tf_static:=/robot/preview/tf_static'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init(args=['--ros-args','-p',f'configuration_dir:={tmp_path / "configuration"}'])
    server, probe = Node('configuration_runtime_fake'), Node('configuration_probe')
    runtime = dict(revision='initial', stage='boot', locked=True, operation_id='', previous='initial')
    profiles = []
    def control(request,response):
        phase = request.phase
        if phase in (0,5):
            runtime.update(previous=runtime['revision'], operation_id=request.operation_id, stage='prepared', locked=True)
            profiles.append(request.configuration)
        if phase in (1,5):
            runtime.update(revision=request.configuration.revision, stage='committed')
        if phase == 2:
            runtime.update(revision=runtime['previous'], stage='rolled_back')
        if phase == 3:
            runtime.update(stage='finished', locked=False)
        response.accepted = True
        for key in ('revision','stage','locked','operation_id'):
            setattr(response,key,runtime[key])
        return response
    server.create_service(ConfigureRobot,'/robot/configuration/control',control)
    node = ConfigurationNode()
    client = probe.create_client(ConfigurationCommand,'/robot/configuration/command')
    motion_settings = []
    probe.create_subscription(MotionSettings, '/robot/motion/settings', lambda value: motion_settings.append(value), 10)
    transforms = {}
    probe.create_subscription(TFMessage,'/robot/preview/tf',lambda message:transforms.update({item.child_frame_id:item for item in message.transforms}),10)
    publisher = probe.create_publisher(JointState,'/robot/preview/joint_states',10)
    executor = MultiThreadedExecutor(num_threads=4)
    for item in (server,probe,node):executor.add_node(item)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def command(operation,**values):
        future=client.call_async(ConfigurationCommand.Request(operation=operation,request_json=json.dumps(values)))
        done=threading.Event();future.add_done_callback(lambda _:done.set())
        assert done.wait(5),'Brak odpowiedzi ConfigurationCommand'
        response=future.result();return response.status_code,json.loads(response.response_json)
    try:
        assert client.wait_for_service(timeout_sec=5)
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            _,state=command('status')
            if state['operation']['phase']=='idle':break
            time.sleep(.1)
        assert state['operation']['phase']=='idle',state
        assert command('system_get')[1]['ignore_missing_drivers'] is False
        assert command('system_set',ignore_missing_drivers=True)[0] == 200
        assert command('system_get')[1]['ignore_missing_drivers'] is True
        assert command('system_get')[1]['allow_missing_motion'] is False
        code, saved = command('system_set', allow_missing_motion=True)
        assert code == 200 and saved['ignore_missing_drivers'] and saved['allow_missing_motion']
        deadline = time.monotonic()+3
        while time.monotonic()<deadline and not any(value.revision==saved['revision'] and value.allow_missing_motion for value in motion_settings):
            time.sleep(.02)
        assert any(value.revision==saved['revision'] and value.allow_missing_motion for value in motion_settings)
        assert command('system_set', ignore_missing_drivers=False)[1]['allow_missing_motion'] is True
        assert command('status')[1]['active']['revision'] == 'initial'
        document=empty_document();document['name']='ROS integracja'
        for key,value in dict(gear_ratio=50,direction=-1,zero_offset=10,lower=-180,upper=180,velocity=30,acceleration=60).items():
            document['values'][f'joint.1.{key}']=value
        document['values']['joint.1.origin_x']=123
        code,snapshot=command('create',document=document);assert code==201
        assert command('get',id=snapshot['id'])[1]['values']['joint.1.origin_x']==123
        assert command('status')[1]['active']['revision']=='initial','Snapshot nie stosuje konfiguracji'
        assert command('apply',document=document,expected_revision='initial')[0]==202
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            _,state=command('status')
            if state['operation']['phase'] in ('done','failed','uncertain'):break
            time.sleep(.1)
        assert state['operation']['phase']=='done',state
        revision=state['active']['revision']
        assert revision==runtime['revision'] and not runtime['locked']
        assert profiles[-1].axes[0].velocity_rad_s==math.radians(30)
        assert profiles[-1].axes[0].zero_offset_rad==math.radians(10)
        assert not profiles[-1].axes[1].configured
        saved=json.loads((tmp_path/'configuration/active.json').read_text())
        assert saved['revision']==revision and saved['document']['values']['joint.1.origin_x']==123
        assert node.coordinator.transport.description==build_urdf(document,revision)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if 'preview/link_1' in transforms and math.isclose(transforms['preview/link_1'].transform.translation.x,.123,abs_tol=1e-9):
                break
            sample=JointState(name=['joint_1'],position=[0.]);sample.header.stamp=probe.get_clock().now().to_msg();publisher.publish(sample);time.sleep(.05)
        assert math.isclose(transforms['preview/link_1'].transform.translation.x,.123,abs_tol=1e-9)
        assert command('apply',document=document,expected_revision='initial')[0]==409
        # Sam zapis tych samych wartości nie zmienia rewizji/modelu ani nie wysyła PREPARE.
        profile_count = len(profiles)
        document['created_at'] = '2026-10-25T01:00:00Z'
        for _ in range(2):
            assert command('apply', document=document, expected_revision=revision)[0] == 202
            deadline = time.monotonic()+5
            while time.monotonic()<deadline:
                _, state = command('status')
                if state['operation']['phase'] == 'done':
                    break
                time.sleep(.02)
            assert state['operation']['success'] is True and state['active']['revision'] == revision
        assert len(profiles) == profile_count and runtime['revision'] == revision
        assert json.loads((tmp_path/'configuration/active.json').read_text()) == saved
        assert node.coordinator.transport.description == build_urdf(saved['document'], revision)
    finally:
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        for item in (node,probe,server):item.destroy_node()
        rclpy.shutdown();process.terminate()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
