"""Settings validation and real headless worker lifecycle (no HTTP socket)."""
import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from simulation_ui import DEFAULTS, Engine, model_description, validate_settings, p, paths

class UiTest(unittest.TestCase):
    def test_settings_reject_invalid_without_mutating(self):
        for values in ({'vx':301},{'vy':-301},{'yaw_rate':60.01},{'gravity':float('nan')},{'gait':'unknown'},
                       {'yaw':200},{'friction':0},{'camera_yaw':True},{'new':1}):
            with self.assertRaises(ValueError):validate_settings(values,DEFAULTS)
        accepted=validate_settings({'vx':300,'vy':-300,'yaw_rate':60},DEFAULTS)
        self.assertEqual((accepted['vx'],accepted['vy'],accepted['yaw_rate']),(300,-300,60))
        self.assertEqual(DEFAULTS['vx'],100)
        self.assertEqual(validate_settings({'gait':'wave','gravity':3.71},DEFAULTS)['gravity'],3.71)

    def test_urdf_hierarchy_matches_measured_bullet_links(self):
        import math
        connection=p.connect(p.DIRECT)
        try:
            robot=p.loadURDF(str(paths.default_urdf()),[.2,-.1,.3],p.getQuaternionFromEuler([.2,-.1,.7]))
            for i in range(p.getNumJoints(robot)):
                if p.getJointInfo(robot,i)[2]==p.JOINT_REVOLUTE:p.resetJointState(robot,i,.1*math.sin(i))
            model,assets=model_description(robot)
            self.assertEqual(len(assets),11)
            self.assertEqual(len([j for j in model['joints'] if j['channel'] is not None]),18)
            indices={p.getJointInfo(robot,i)[12].decode():i for i in range(p.getNumJoints(robot))}
            joints=[p.getJointState(robot,i)[0] for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot,i)[2]==p.JOINT_REVOLUTE]
            dynamics=p.getDynamicsInfo(robot,-1)
            root=p.multiplyTransforms(*p.getBasePositionAndOrientation(robot),*p.invertTransform(dynamics[3],dynamics[4]))
            frames={model['root']:root}
            pending=list(model['joints'])
            while pending:
                ready=[j for j in pending if j['parent'] in frames]
                self.assertTrue(ready)
                for j in ready:
                    frame=p.multiplyTransforms(*frames[j['parent']],j['position'],j['quaternion'])
                    if j['channel'] is not None:
                        frame=p.multiplyTransforms(*frame,[0,0,0],p.getQuaternionFromAxisAngle(j['axis'],joints[j['channel']]))
                    measured=p.getLinkState(robot,indices[j['child']],computeForwardKinematics=True)
                    self.assertLess(math.dist(frame[0],measured[4]),1e-5,j['child'])
                    self.assertAlmostEqual(abs(sum(a*b for a,b in zip(frame[1],measured[5]))),1,places=5)
                    frames[j['child']]=frame;pending.remove(j)
        finally:p.disconnect(connection)

    def test_brain_http_latched_motion_and_invalid_input(self):
        import json
        from http.server import ThreadingHTTPServer
        from threading import Thread
        from urllib.error import HTTPError
        from urllib.request import Request, urlopen
        from simulation_ui import make_handler
        engine=Engine()
        try:
            server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(engine))
        except Exception:
            engine.closed.set();engine.worker.join(10)
            raise
        thread=Thread(target=server.serve_forever,daemon=True);thread.start()
        url=f'http://127.0.0.1:{server.server_port}/api/brain'
        def send(raw):
            request=Request(url,json.dumps(raw).encode(),headers={'X-Muha-UI':'1','Content-Type':'application/json'})
            with urlopen(request,timeout=3) as response:return response.status
        def wait_for(predicate,seconds=15):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                with engine.lock:state=dict(engine.snapshot)
                self.assertNotIn('error',state)
                if predicate(state):return state
                time.sleep(.05)
            self.fail(str(state))
        try:
            self.assertEqual(send({'command':'forward'}),202)
            wait_for(lambda s:s.get('output',{}).get('state')=='WALK')
            time.sleep(2.2)  # No UI heartbeat or repeated brain command.
            with engine.lock:state=dict(engine.snapshot)
            self.assertEqual(state['output']['state'],'WALK')
            self.assertTrue(state['brain']['active'])
            send({'command':'jump'})
            wait_for(lambda s:s['brain']['boost'])
            send({'command':'stop'})
            wait_for(lambda s:s['output']['state']=='IDLE')
            send({'command':'left'})
            wait_for(lambda s:s['output']['state']=='WALK')
            with self.assertRaises(HTTPError) as error:send({'command':[]})
            self.assertEqual(error.exception.code,400)
            error.exception.close()
            wait_for(lambda s:s['brain']['blocked'] and s['output']['state']=='IDLE')
            send({'command':'forward'})
            time.sleep(.2)
            with engine.lock:self.assertTrue(engine.snapshot['brain']['blocked'])
            send({'command':'start'})
            wait_for(lambda s:not s['brain']['blocked'])
            send({'command':'backward'})
            wait_for(lambda s:s['output']['state']=='WALK')
            engine.submit({'action':'stop'})
            wait_for(lambda s:not s['brain']['active'] and s['output']['state']=='IDLE')
        finally:
            server.shutdown();server.server_close();thread.join(3)
            engine.closed.set();engine.worker.join(10)

    def test_worker_lifecycle(self):
        from unittest.mock import patch
        no_renderer=patch('simulation_ui.p.getCameraImage',side_effect=AssertionError('CPU renderer must not run'))
        no_renderer.start()
        engine=Engine()
        def wait_for(predicate,seconds=45):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                engine.submit({'action':'heartbeat'})
                with engine.lock:s=dict(engine.snapshot)
                if s.get('error'):self.fail(s['error'])
                if predicate(s):return s
                time.sleep(.1)
            self.fail('Timed out: '+str(s))
        try:
            wait_for(lambda s:s.get('output',{}).get('state')=='IDLE')
            engine.submit({'action':'apply','settings':{'gait':'wave','vx':15,'friction':.6}})
            wait_for(lambda s:s.get('output',{}).get('gait')=='wave')
            engine.submit({'action':'move'})
            wait_for(lambda s:s.get('output',{}).get('state')=='WALK')
            engine.submit({'action':'pause'})
            state=wait_for(lambda s:s.get('paused'))
            initial=state['output']['timestamp']
            time.sleep(.4)
            with engine.lock:self.assertEqual(engine.snapshot['output']['timestamp'],initial)
            engine.submit({'action':'stop'})
            wait_for(lambda s:s.get('output',{}).get('state')=='IDLE')
            engine.submit({'action':'reset','settings':{'yaw':90,'gravity':3.71}})
            state=wait_for(lambda s:s.get('settings',{}).get('yaw')==90)
            self.assertLess(state['output']['timestamp'],initial)
            self.assertEqual(state['settings']['gravity'],3.71)
            self.assertEqual(len(state['pose']['joints']),18)
            self.assertEqual(len(state['pose']['quaternion']),4)
        finally:
            engine.closed.set();engine.worker.join(10)
            self.assertFalse(engine.worker.is_alive())
            no_renderer.stop()

if __name__=='__main__':unittest.main()
