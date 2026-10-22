from pathlib import Path
from dataclasses import asdict
import argparse,json,signal,sys,time
import xml.etree.ElementTree as ET
from .bootstrap import prepare
ROOT=prepare()
from .robot_platform import RobotSettings,RobotCameraEngine,CAMERAS,platform_xml,SOURCES


def main():
    parser=argparse.ArgumentParser(description='固定站姿 A2-W 尺寸模型与三相机平台')
    parser.add_argument('--scene-dir',type=Path,default=ROOT.parent if (ROOT.parent/'scene.xml').exists() else Path.home()/'E_dog/JiFang_world')
    parser.add_argument('--config',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--no-ros',action='store_true')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--frames',type=int,default=0)
    parser.add_argument('--export-model',type=Path)
    args,ros_args=parser.parse_known_args()
    if ros_args and '--ros-args' not in ros_args:parser.error('未知参数: '+' '.join(ros_args))
    if args.frames<0:parser.error('--frames 不可为负数')
    settings=RobotSettings().updated(json.loads(args.config.read_text())) if args.config else RobotSettings().validate()
    if args.export_model:
        args.export_model.parent.mkdir(parents=True,exist_ok=True)
        root=platform_xml(args.scene_dir,settings);ET.indent(root,space='  ');ET.ElementTree(root).write(args.export_model,encoding='utf-8',xml_declaration=True)
        print('已导出站姿场景：',args.export_model);return
    output=args.output or args.scene_dir/'camera_lab/robot_experiments'
    engine=RobotCameraEngine(args.scene_dir,settings);node=executor=None
    try:
        if not args.no_ros:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from .robot_ros import PlatformNode
            rclpy.init(args=ros_args);node=PlatformNode(engine);executor=SingleThreadedExecutor();executor.add_node(node)
        if args.headless:
            from PIL import Image
            from datetime import datetime
            from zoneinfo import ZoneInfo
            count=0
            while not args.frames or count<args.frames:
                start=time.perf_counter()
                if executor:
                    for _ in range(4):executor.spin_once(timeout_sec=0)
                images={};results={}
                for name in CAMERAS:images[name],results[name]=engine.capture_camera(name)
                if node:node.publish(images,results)
                count+=1
                if count==1:print('三路相机就绪：', {name:results[name]['bottom_three_decoded'] for name in CAMERAS},flush=True)
                if args.frames and count==args.frames:
                    directory=output/('platform_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'));directory.mkdir(parents=True)
                    for name in CAMERAS:
                        Image.fromarray(images[name]).save(directory/(name+'.png'))
                        (directory/(name+'_result.json')).write_text(json.dumps(results[name],indent=2)+'\n')
                    (directory/'settings.json').write_text(json.dumps(asdict(engine.platform),indent=2)+'\n')
                    print('已保存三路图像：',directory,flush=True)
                time.sleep(max(0.,1/engine.platform.fps-(time.perf_counter()-start)))
        else:
            from PyQt5 import QtCore,QtWidgets
            from .robot_gui import RobotWindow
            app=QtWidgets.QApplication([sys.argv[0]]);app.setStyle('Fusion');font=app.font();font.setPointSize(10);app.setFont(font)
            window=RobotWindow(engine,node,executor,output);window.show();signal.signal(signal.SIGINT,lambda *unused:window.close())
            timer=QtCore.QTimer();timer.timeout.connect(lambda:None);timer.start(250);app.exec_()
    except KeyboardInterrupt:pass
    finally:
        engine.close()
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if not args.no_ros:
            import rclpy
            if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
