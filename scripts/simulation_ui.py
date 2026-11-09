"""Local-only browser interface to the real pinned PyBullet/controller stack."""
import argparse
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from motion import MotionController, get_motion_command, PERIOD, load_config
from simulation_sink import SimulationSink, URDF_YAW_OFFSET
from simulation import paths
import pybullet as p
import pybullet_data

DEFAULTS = dict(vx=20., vy=0., yaw_rate=0., gait='tripod', yaw=0., friction=.8,
                gravity=9.81, camera_yaw=40., camera_pitch=-30., distance=.9)
SAFETY = load_config(ROOT / 'config/demo.yml')['safety']
BOUNDS = dict(vx=(-SAFETY['lin_vel_max'],SAFETY['lin_vel_max']),
              vy=(-SAFETY['lin_vel_max'],SAFETY['lin_vel_max']),
              yaw_rate=(-SAFETY['ang_vel_max'],SAFETY['ang_vel_max']), yaw=(-180,180),
              friction=(.1,1.5), gravity=(1,15), camera_yaw=(-180,180),
              camera_pitch=(-80,-10), distance=(.45,1.8))


def validate_settings(raw, current):
    if not isinstance(raw, dict) or set(raw) - set(DEFAULTS):
        raise ValueError('Неизвестные параметры')
    result = dict(current)
    for key, value in raw.items():
        if key == 'gait':
            if value not in ('tripod','wave','ripple'):
                raise ValueError('Неизвестная походка')
        elif isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or not BOUNDS[key][0] <= value <= BOUNDS[key][1]:
            raise ValueError(f'{key}: допустимый диапазон {BOUNDS[key]}')
        result[key] = value
    return result


def model_description(robot):
    """Expose the original URDF visual hierarchy, without loading a renderer."""
    tree = ET.parse(paths.default_urdf()).getroot()
    materials = {m.get('name'): [float(x) for x in m.find('color').get('rgba').split()]
                 for m in tree.findall('material') if m.find('color') is not None}
    channels = {p.getJointInfo(robot, i)[1].decode(): n for n, i in enumerate(
        i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot,i)[2]==p.JOINT_REVOLUTE)}
    def origin(element):
        o = element.find('origin')
        xyz = [float(x) for x in o.get('xyz','0 0 0').split()] if o is not None else [0,0,0]
        rpy = [float(x) for x in o.get('rpy','0 0 0').split()] if o is not None else [0,0,0]
        return xyz, p.getQuaternionFromEuler(rpy)
    assets = []
    links = []
    for link in tree.findall('link'):
        visuals=[]
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None: raise ValueError('Only mesh visuals are supported')
            filename=mesh.get('filename')
            if filename not in assets: assets.append(filename)
            material=visual.find('material')
            color=material.find('color') if material is not None else None
            rgba=([float(x) for x in color.get('rgba').split()] if color is not None else
                  materials.get(material.get('name'),[.3,.3,.3,1]) if material is not None else [.3,.3,.3,1])
            pos,rot=origin(visual)
            visuals.append(dict(mesh=assets.index(filename),scale=[float(x) for x in mesh.get('scale','1 1 1').split()],
                                position=pos,quaternion=rot,color=rgba))
        links.append(dict(name=link.get('name'),visuals=visuals))
    joints=[]
    for joint in tree.findall('joint'):
        pos,rot=origin(joint);axis=joint.find('axis')
        joints.append(dict(parent=joint.find('parent').get('link'),child=joint.find('child').get('link'),
                           position=pos,quaternion=rot,axis=[float(x) for x in axis.get('xyz').split()] if axis is not None else [0,0,1],
                           channel=channels.get(joint.get('name'))))
    children={j['child'] for j in joints}
    roots=[link['name'] for link in links if link['name'] not in children]
    if len(roots)!=1:raise ValueError('URDF must have one root')
    return dict(root=roots[0],links=links,joints=joints,meshes=['/mesh/'+str(i) for i in range(len(assets))]), assets


class Engine:
    """All firmware and Bullet access lives on one worker thread."""
    def __init__(self):
        self.commands = queue.Queue(maxsize=32)
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.closed = threading.Event()
        self.settings = dict(DEFAULTS)
        self.epoch = 0
        self.model = {}
        self.assets = []
        self.snapshot = {'ready':False}
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        if not self.ready.wait(30):
            raise RuntimeError('Simulation initialization timed out')

    def submit(self, request):
        if not isinstance(request,dict) or request.get('action') not in ('apply','start','move','stop','pause','reset','heartbeat'):
            raise ValueError('Неизвестная команда')
        # Synchronous validation prevents the interface from reporting false success.
        with self.lock:
            if request['action'] in ('apply','reset'):
                request = {**request, 'settings':validate_settings(request.get('settings',{}), self.snapshot.get('settings',DEFAULTS))}
        try:
            self.commands.put_nowait(request)
        except queue.Full:
            raise ValueError('Очередь занята; повторите команду')

    def reset(self):
        self.epoch += 1
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-self.settings['gravity'])
        p.setTimeStep(.004)
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        self.plane = p.loadURDF('plane.urdf')
        p.changeDynamics(self.plane,-1,lateralFriction=self.settings['friction'])
        self.robot = p.loadURDF(str(paths.default_urdf()),[0,0,.20],
                               p.getQuaternionFromEuler([0,0,math.radians(self.settings['yaw'])+URDF_YAW_OFFSET]),
                               flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.model, self.assets = model_description(self.robot)
        self.controller = MotionController()
        self.sink = SimulationSink(p,self.robot,self.controller.config)
        self.mode = 'start'
        self.moving = False
        self.paused = False
        self.last_client = time.monotonic()
        self.notice = 'Подъём в стойку'
        self.last_output = None

    def publish(self):
        pos, rot = p.getBasePositionAndOrientation(self.robot)
        # Bullet base pose is its inertial COM frame; the browser needs URDF root.
        dynamics=p.getDynamicsInfo(self.robot,-1)
        inv=p.invertTransform(dynamics[3],dynamics[4])
        base_pos,base_rot=p.multiplyTransforms(pos,rot,*inv)
        joints=[state[0] for state in p.getJointStates(self.robot,self.sink.interface.joints)]
        with self.lock:
            self.snapshot={'ready':True,'bounds':BOUNDS,'settings':dict(self.settings),'output':asdict(self.last_output) if self.last_output else None,
                           'position':pos,'rpy':p.getEulerFromQuaternion(rot),'paused':self.paused,
                           'moving':self.moving,'notice':self.notice,'epoch':self.epoch,
                           'pose':{'position':base_pos,'quaternion':base_rot,'joints':joints},
                           'wall_time':time.monotonic()}
        self.ready.set()

    def run(self):
        connection=p.connect(p.DIRECT)
        try:
            self.reset()
            while not self.closed.is_set():
                started=time.monotonic()
                while True:
                    try: req=self.commands.get_nowait()
                    except queue.Empty: break
                    action=req['action']
                    self.last_client=started
                    if action=='heartbeat':continue
                    if action in ('apply','reset'):
                        settings=req['settings']
                        if action=='apply' and settings['yaw']!=self.settings['yaw']:
                            self.notice='Начальный угол изменяется кнопкой «Новая сцена»'
                            continue
                        self.settings=settings
                        if action=='reset':
                            self.reset()
                        else:
                            p.setGravity(0,0,-self.settings['gravity'])
                            p.changeDynamics(self.plane,-1,lateralFriction=self.settings['friction'])
                            self.notice='Параметры применены'
                    elif action=='start':
                        self.mode='start';self.moving=False;self.paused=False;self.notice='Разрешение движения'
                    elif action=='move':
                        self.moving=True;self.paused=False;self.notice='Команда движения'
                    elif action=='stop':
                        self.moving=False;self.paused=False;self.notice='Остановка в стойке'
                    elif action=='pause':
                        self.paused=not self.paused;self.notice='Физика приостановлена' if self.paused else 'Физика продолжается'
                if started-self.last_client>2:
                    self.moving=False
                    self.notice='Нет связи с интерфейсом; движение остановлено'
                if not self.paused:
                    desired=self.settings['gait']
                    changing=desired!=self.controller.gait
                    gait=desired if self.controller.state in ('OFF','IDLE') else self.controller.gait
                    mode=self.mode or ('move' if self.moving and not changing else 'stop')
                    self.mode=None
                    if changing:self.notice='Остановка перед сменой походки'
                    s=self.settings
                    command=get_motion_command(self.controller.time,mode,s['vx']/1000,s['vy']/1000,math.radians(s['yaw_rate']),gait=gait)
                    self.last_output=self.controller.update_motion(command,PERIOD)
                    self.sink.apply(self.last_output)
                    for _ in range(5):p.stepSimulation()
                    if not changing and self.notice=='Остановка перед сменой походки':
                        self.notice='Походка применена'
                    if self.last_output.state=='IDLE' and self.notice=='Подъём в стойку':
                        self.notice='Готов к движению'
                    if not self.last_output.armed:
                        self.moving=False;self.notice=self.last_output.diagnostic
                self.publish()
                self.closed.wait(max(0,PERIOD-(time.monotonic()-started)))
        except Exception as exc:
            with self.lock:self.snapshot={'ready':False,'error':str(exc)}
            self.ready.set()
        finally:
            p.disconnect(connection)


def make_handler(engine):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,status,body,kind):
            self.send_response(status)
            self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            path=self.path.split('?')[0]
            if path=='/':return self.send(200,(ROOT/'ui/index.html').read_bytes(),'text/html; charset=utf-8')
            if path=='/api/state':
                with engine.lock:body=json.dumps(engine.snapshot,allow_nan=False).encode()
                return self.send(200,body,'application/json')
            if path=='/viewer.js':
                return self.send(200,(ROOT/'ui/viewer.js').read_bytes(),'text/javascript; charset=utf-8')
            if path=='/api/model':
                return self.send(200,json.dumps(engine.model).encode(),'application/json')
            if path.startswith('/mesh/'):
                index=path.removeprefix('/mesh/')
                if index.isdigit() and int(index)<len(engine.assets):
                    asset=paths.default_urdf().parent/engine.assets[int(index)]
                    return self.send(200,asset.read_bytes(),'application/octet-stream')
            self.send(404,b'Not found','text/plain')
        def do_POST(self):
            if self.path!='/api/command':return self.send(404,b'Not found','text/plain')
            if self.headers.get('X-Muha-UI')!='1':return self.send(403,b'Forbidden','text/plain')
            origin=self.headers.get('Origin')
            if origin and origin not in (f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'):
                return self.send(403,b'Forbidden origin','text/plain')
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=8192:raise ValueError('Неверный размер запроса')
                engine.submit(json.loads(self.rfile.read(size)))
                self.send(202,b'{"accepted":true}','application/json')
            except (ValueError,TypeError) as exc:
                self.send(400,json.dumps({'error':str(exc)},ensure_ascii=False).encode(),'application/json')
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    engine=Engine()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(engine))
    print(f'Железная муха: http://127.0.0.1:{server.server_port}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        server.server_close();engine.closed.set();engine.worker.join(timeout=5)

if __name__=='__main__':main()
