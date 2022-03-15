#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import os
import sys
import subprocess
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from rovi_utils import tflib

Param={
  "enable":0,
  "box0_width":60,
  "box0_depth":40,
  "box0_points":500,
  "box0_low":100,
  "box0_crop":200,
  "bucket_width":870
}
Config={
#  "axis_frame_ids":["bucket_post0","bucket_post1"]
  "axis_frame_ids":["prepro"]
}

do1busy=False
P0=np.array([]).reshape((-1,3))
Scene=P0
SceneP=None
SceneN=0
ParamN=''

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def Pnul():
  return np.reshape([],(-1,3))

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def pFrame(P0,Px):
  RT=np.eye(4)
  ex=Px-P0
  xlen=np.linalg.norm(ex)
  ex=ex/xlen
  ez=np.array([0,0,1])
  ey=np.cross(ez,ex)
  RT[0,0:3]=ex
  RT[1,0:3]=ey
  RT[2,0:3]=ez
  RT[3,0:3]=P0
  RT[3,2]=0
  RT=RT.T
  tfs=TransformStamped()
  tfs.header.frame_id="world"
  tfs.child_frame_id="prepro"
  tfs.transform=tflib.fromRT(RT)
  broadcaster.sendTransform([tfs])
  return RT

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def pCollectPeakX(pc,dx,dz):
  pitch = 5
  cnt=[]
  codx=[]
  codz=[]
  cpre=0
  inc=True
  hx0=np.min(pc.T[0])
  hx1=np.max(pc.T[0])
  hz0=np.min(pc.T[2])
  hz1=np.max(pc.T[2])
  for x in np.arange(hx0,hx1,pitch):
    pc1=pc[ np.ravel(np.abs(pc[:,0]-x)<dx) ]
    cmaxn=0
    cmaxz=0
    for z in np.arange(hz0,hz1,pitch):
      n=len(pc1[ np.ravel(np.abs(pc1[:,2]-z)<dz) ])
      if cmaxn<n:
        cmaxn=n
        cmaxz=z
    if inc:
      if cpre>cmaxn:
        if cpre>Param["box0_low"]:
          cnt.append(cpre)
          codx.append(x-pitch)
          codz.append(cmaxz)
        inc=False
    else:
      if cpre<cmaxn: inc=True
    cpre=cmaxn
  return cnt,codx,codz

def pScanX(pc,h,wid,dep,thres):
  pc=pc[ np.ravel(pc[:,1]<0) ] # Y<0
  if len(pc)<thres: return 0,pc,0,0,0

  cnt,coordx,coordz=pCollectPeakX(pc,wid/2,dep/2)  #Collects PC peak
  print("scanX",cnt,coordx,coordz)
  peaks=PoseArray()
  peaks.header.frame_id='prepro'
  if len(coordx)>0:
    cnt=np.array(cnt)
    coordx=np.array(coordx)
    coordz=np.array(coordz)
    sel=cnt>thres
    if sum(sel)>0:
      cnt=cnt[sel]
      coordx=coordx[sel]
      n=np.argmin(np.abs(coordx-h/2))
      for x,z in zip(coordx,coordz):
         p=Pose()
         p.position.x=x
         p.position.y=0
         p.position.z=z
         p.orientation.x=0
         p.orientation.y=0
         p.orientation.z=-0.707
         p.orientation.w=0.707
         peaks.poses.append(p)
      pub_peaks.publish(peaks)
      return sum(sel),pc,cnt[n],coordx[n],coordz[n]
    else:
      n=np.argmax(cnt)
      pub_peaks.publish(peaks)
      return 0,pc,cnt[n],coordx[n],coordz[n]
  else:
    pub_peaks.publish(peaks)
    return 0,pc,0,0,0

def pCropY(pc,y0,yh):
  cy=RT[1,3]/2

def done1(report):
  global Scene,SceneP,do1busy
  if len(report)>0:
    msg=String()
    msg.data=str(report)
    pub_str.publish(msg)
  if do1busy:
    do1busy=False
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_done1.publish(mTrue),oneshot=True)

def cb_ps(msg):
  global Scene,SceneP,SceneN,ParamN,do1busy
  try:
    Param.update(rospy.get_param("/prepro"))
  except Exception as e:
    print("get_param exception:",e.args)
  Scene=np.reshape(msg.data,(-1,3))
  if SceneN==len(Scene) and ParamN==str(Param):
    print("prepro::cb_ps unchaged")
    done1({})
    return
  SceneN=len(Scene)
  ParamN=str(Param)
  if SceneN<100:
    done1({'probables':0,'prob_v':0,'prob_m':0,'prob_x':0,'prob_z':1000})
    SceneP=Scene
    pub_ps.publish(np2F(SceneP))
    return
  if Param["enable"]==0:
    done1({'probables':1,'prob_v':0,'prob_m':100,'prob_x':0,'prob_z':1000})
    SceneP=Scene
    pub_ps.publish(np2F(SceneP))
    return
  pub_ps.publish(np2F(Scene))
  if len(Config["axis_frame_ids"])>1:
    P0=getRT("world",Config["axis_frame_ids"][0])[0:3,3].ravel()
    Px=getRT("world",Config["axis_frame_ids"][1])[0:3,3].ravel()
    wTu=pFrame(P0,Px)
    wTc=getRT("world","camera/capture0")
    uTc=np.linalg.inv(wTu).dot(wTc)
    xlen=np.linalg.norm(Px-P0)
  else:
    uTc=getRT(Config["axis_frame_ids"][0],"camera/capture0")
    xlen=Param["bucket_width"]
  cTu=np.linalg.inv(uTc)
  uscn=pTr(uTc,Scene)
  pbn,uscn0,cnt0,cx0,cz0=pScanX(uscn,xlen,Param["box0_width"],Param["box0_depth"],Param["box0_points"])  #Scan PC peak of F-end of workpiece
  report={}
  report["probables"]=pbn
  report["prob_v"]=cnt0
  if pbn>0: #probable point clusters
    report["prob_x"]=cx0-uTc[0,3]
    report["prob_z"]=uTc[2,3]-cz0
    margin=cx0 if cx0<xlen/2 else xlen-cx0
    report["prob_m"]=margin
    uscn1=uscn[ np.ravel(np.abs(uscn[:,0]-cx0)<Param["box0_crop"]/2) ]
    SceneP=pTr(cTu,uscn1)
    print("cb_ps done",len(SceneP),do1busy)
  else:
    print("cb_ps done None")
    SceneP=Pnul()
  done1(report)

def cb_do1free(msg):
  global do1busy
  do1busy=False

def cb_do1(msg):
  global do1busy
  if not do1busy:
    do1busy=True
    rospy.Timer(rospy.Duration(5),cb_do1free,oneshot=True)

def cb_do2(msg):
  global SceneP
  if Param["enable"]:
    pub_ps.publish(np2F(SceneP))
  rospy.Timer(rospy.Duration(0.1),lambda ev: pub_done2.publish(mTrue),oneshot=True)

def cb_save(msg):
  global Scene
  pc=o3d.geometry.PointCloud()
  pc.points=o3d.utility.Vector3dVector(Scene)
  o3d.io.write_point_cloud("/tmp/prepro.ply",pc)

########################################################
rospy.init_node("prepro",anonymous=True)
thispath=subprocess.getoutput("rospack find rovi_sim")
###Load params
try:
  Config.update(rospy.get_param("/config/prepro"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("~do1",Bool,cb_do1)  #search cluster
rospy.Subscriber("~do2",Bool,cb_do2)  #crop
rospy.Subscriber("/prepro/save",Bool,cb_save)
pub_ps=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_str=rospy.Publisher("/report",String,queue_size=1)
pub_done1=rospy.Publisher("~done1",Bool,queue_size=1)
pub_done2=rospy.Publisher("~done2",Bool,queue_size=1)
pub_peaks=rospy.Publisher("/prepro/peaks",PoseArray,queue_size=1)
###Bool message
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
