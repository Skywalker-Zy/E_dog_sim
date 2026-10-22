from dataclasses import asdict
from pathlib import Path
import argparse
import json
import signal
import sys
import time
from .bootstrap import prepare, default_scene_dir
PACKAGE_ROOT = prepare()
from .core import CameraEngine, Settings, load_settings
from .experiments import save_snapshot, save_scan, save_sweep


def main(argv=None):
    parser=argparse.ArgumentParser(description='机房相机视角、二维码与 ROS 2 实验平台')
    default_scene=default_scene_dir()
    parser.add_argument('--scene-dir',type=Path,default=default_scene)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--no-ros',action='store_true')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--frames',type=int,default=0,help='无窗口模式发布多少帧，0 为持续运行')
    parser.add_argument('--scan',action='store_true',help='无窗口自动巡检 35 个采样位姿')
    parser.add_argument('--sweep',action='store_true',help='比较 60/90/120 度以及 1080p/4K，输出报告后退出')
    parser.add_argument('--sweep-fovs',default='60,90,120')
    parser.add_argument('--sweep-resolutions',default='1920x1080,3840x2160')
    args,ros_args=parser.parse_known_args(argv)
    if ros_args and '--ros-args' not in ros_args:parser.error('无法识别的参数: '+' '.join(ros_args))
    if args.frames<0:parser.error('--frames 不可为负数')
    settings=load_settings(args.config)
    output=args.output or args.scene_dir.parent/'02_single_camera/experiments'
    engine=CameraEngine(args.scene_dir,settings)
    node=executor=None
    app=None
    try:
        if not args.no_ros:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from .ros_bridge import CameraNode
            rclpy.init(args=ros_args)
            node=CameraNode(engine);executor=SingleThreadedExecutor();executor.add_node(node)
        if args.headless or args.scan or args.sweep:
            headless(args,engine,node,executor,output)
        else:
            from PyQt5 import QtCore,QtWidgets
            from .gui import Window
            app=QtWidgets.QApplication([sys.argv[0]])
            app.setStyle('Fusion')
            font=app.font();font.setPointSize(10);app.setFont(font)
            window=Window(engine,node,executor,output)
            window.show()
            signal.signal(signal.SIGINT,lambda *unused:window.close())
            exit_timer=QtCore.QTimer();exit_timer.timeout.connect(lambda:None);exit_timer.start(250)
            app.exec_()
    finally:
        engine.close()
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if not args.no_ros:
            import rclpy
            if rclpy.ok():rclpy.shutdown()


def headless(args,engine,node,executor,output):
    if args.sweep and not engine.settings.detect_enabled:raise ValueError('批量实验需要 detect_enabled=true')
    if args.sweep:
        comparisons=[]
        for resolution in args.sweep_resolutions.split(','):
            width,height=map(int,resolution.lower().split('x'))
            for fov in map(float,args.sweep_fovs.split(',')):
                settings=engine.settings.updated(dict(width=width,height=height,hfov_deg=fov,lens_mode='fov'))
                engine.apply(settings)
                records=scan(engine,node,executor)
                directory,summary=save_scan(output,records,settings,label=f'scan_{width}x{height}_h{fov:g}')
                comparisons.append(dict(width=width,height=height,hfov_deg=fov,decoded_count=len(summary['decoded_indices']),total=35,coverage=summary['coverage'],max_single_frame=summary['max_single_frame'],frames=len(records),processing_seconds=summary['processing_seconds'],missing_indices=summary['missing_indices'],report=str(directory)))
                print(json.dumps(comparisons[-1]),flush=True)
        print('对比报告：',save_sweep(output,comparisons),flush=True)
    elif args.scan:
        if not engine.settings.detect_enabled:raise ValueError('自动巡检需要 detect_enabled=true')
        initial=engine.settings
        records=scan(engine,node,executor)
        directory,summary=save_scan(output,records,initial)
        print('巡检报告：',directory,'识别',len(summary['decoded_indices']),'/35',flush=True)
    else:
        count=0
        try:
            while args.frames==0 or count<args.frames:
                start=time.perf_counter()
                if executor:
                    for _ in range(4):executor.spin_once(timeout_sec=0)
                image,result=engine.capture()
                if node:node.publish(image,result)
                count+=1
                if args.frames and count==args.frames:
                    directory=save_snapshot(output,image,result)
                    print('图像与参数：',directory,flush=True)
                if count==1:print('相机就绪；当前解码',len(result['decoded_indices']),'/35',flush=True)
                time.sleep(max(0.,1/engine.settings.fps-(time.perf_counter()-start)))
        except KeyboardInterrupt:pass


def scan(engine,node,executor):
    initial=engine.settings
    engine.cumulative.clear()
    records=[]
    if node:node.command_locked=True
    try:
        for index in range(1,36):
            settings=engine.aim(index)
            if node:node.set_settings(asdict(settings),internal=True)
            else:engine.apply(settings)
            image,result=engine.capture()
            records.append(result)
            if node:node.publish(image,result)
            if executor:
                for _ in range(4):executor.spin_once(timeout_sec=0)
            if index%5==0:print(f"采样 {index}/35，累计解码 {len(engine.cumulative)}/35",flush=True)
    finally:
        if node:
            node.command_locked=False
            node.set_settings(asdict(initial),internal=True)
        else:engine.apply(initial)
    return records


if __name__=='__main__':main()
