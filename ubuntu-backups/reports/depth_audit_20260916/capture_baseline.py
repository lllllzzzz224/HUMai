import time,json,os
from pathlib import Path
from collections import defaultdict,deque
import numpy as np,cv2,rclpy,tf2_ros
from sensor_msgs.msg import Image,CameraInfo
from rcl_interfaces.srv import ListParameters,GetParameters
from rcl_interfaces.msg import Log
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
OUT=Path(__file__).parent/'baseline';OUT.mkdir(exist_ok=True)
if (OUT/'capture.json').exists():raise RuntimeError('Do not overwrite baseline')
rclpy.init();n=rclpy.create_node('depth_baseline_readonly');bridge=CvBridge();tf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(tf,n)
records=defaultdict(list);infos={};logs=[];params={};start=time.time()
def info(k,m):infos[k]={'frame':m.header.frame_id,'K':list(m.k),'D':list(m.d),'R':list(m.r),'P':list(m.p),'distortion_model':m.distortion_model,'width':m.width,'height':m.height}
def frame(k,m):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9;idx=len(records[k]);a=bridge.imgmsg_to_cv2(m,'bgr8' if k=='rgb' else 'passthrough');name=f'{k}_{idx:04d}'+('.jpg' if k=='rgb' else '.npy')
 if k=='rgb':cv2.imwrite(str(OUT/name),a)
 else:np.save(OUT/name,a)
 records[k].append({'index':idx,'stamp_ns':int(m.header.stamp.sec)*1000000000+int(m.header.stamp.nanosec),'stamp':stamp,'receipt_wall':time.time(),'frame':m.header.frame_id,'encoding':m.encoding,'height':m.height,'width':m.width,'path':name,'nonzero_fraction':float(np.mean(a>0)) if k!='rgb' else None})
subs=[]
for k,topic in [('raw','/camera/camera/depth/image_rect_raw'),('aligned','/camera/camera/aligned_depth_to_color/image_raw'),('rgb','/camera/camera/color/image_raw')]:subs.append(n.create_subscription(Image,topic,lambda m,k=k:frame(k,m),qos_profile_sensor_data))
for k,topic in [('raw','/camera/camera/depth/camera_info'),('aligned','/camera/camera/aligned_depth_to_color/camera_info'),('rgb','/camera/camera/color/camera_info')]:subs.append(n.create_subscription(CameraInfo,topic,lambda m,k=k:info(k,m),qos_profile_sensor_data))
subs.append(n.create_subscription(Log,'/rosout',lambda m:logs.append({'stamp_sec':m.stamp.sec,'stamp_ns':m.stamp.nanosec,'level':m.level,'name':m.name,'msg':m.msg}) if ('camera'in m.name or m.level>=30) else None,100))
try:
 cli=n.create_client(ListParameters,'/camera/camera/list_parameters');assert cli.wait_for_service(timeout_sec=5)
 f=cli.call_async(ListParameters.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5);names=f.result().result.names
 g=n.create_client(GetParameters,'/camera/camera/get_parameters');g.wait_for_service(timeout_sec=5);f=g.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 for k,v in zip(names,f.result().values):params[k]={1:v.bool_value,2:v.integer_value,3:v.double_value,4:v.string_value}.get(v.type,str(v))
 (OUT/'parameters.json').write_text(json.dumps(params,indent=2))
 start=time.time()
 while time.time()-start<35:rclpy.spin_once(n,timeout_sec=.02)
 transforms={}
 for a,b in [('camera_color_optical_frame','camera_depth_optical_frame'),('camera_link','camera_color_optical_frame')]:
  t=tf.lookup_transform(a,b,rclpy.time.Time());v=t.transform.translation;q=t.transform.rotation;transforms[a+'<-'+b]={'xyz':[v.x,v.y,v.z],'xyzw':[q.x,q.y,q.z,q.w]}
 finally_dummy=None
finally:
 (OUT/'capture.json').write_text(json.dumps({'start_wall':start,'duration_s':time.time()-start,'records':dict(records),'camera_info':infos,'transforms':locals().get('transforms',{}),'logs':logs},indent=2))
 print(json.dumps({'counts':{k:len(v)for k,v in records.items()},'duration':time.time()-start,'depth_units':{k:v for k,v in params.items()if 'unit'in k},'logs':len(logs)}));n.destroy_node();rclpy.shutdown()
