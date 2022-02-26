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
from rovi_utils import tflib

Param={
  "enable":0,
  "box0_width":60,
  "box0_depth":40,
  "box0_points":500,
  "box0_low":100,
  "box0_crop":200,
  "post_margin":100
}
Config={
  "axis_frame_ids":["bucket_post0","bucket_post1"]
}

do1cema=0

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

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
  return RT,xlen

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def pCollectPeakX(pc,hx,dx,hz,dz):
  pitch = 5
  cnt=[]
  cod=[]
  cpre=0
  inc=True
  for x in np.arange(0,hx,pitch):
    pc1=pc[ np.ravel(np.abs(pc[:,0]-x)<dx) ]
    cmax=0
    for z in np.arange(0,hz,pitch):
      n=len(pc1[ np.ravel(np.abs(pc1[:,2]-z)<dz) ])
      if cmax<n: cmax=n
    if inc:
      if cpre>cmax:
        if cpre>Param["box0_low"]:
          cnt.append(cpre)
          cod.append(x-pitch)
        inc=False
    else:
      if cpre<cmax: inc=True
    cpre=cmax
  return cnt,cod

def pScanX(pc,h,wid,dep,thres):
  pc=pc[ np.ravel(pc[:,1]<0) ] # Y<0
  cnt,coord=pCollectPeakX(pc,h,wid/2,1000,dep/2)  #Collects PC peak
  print("scanX",cnt,coord)
  if len(coord)>0:
    cnt=np.array(cnt)
    coord=np.array(coord)
    sel=cnt>thres
    if sum(sel)>0:
      cnt=cnt[sel]
      coord=coord[sel]
      n=np.argmin(np.abs(coord-h/2))
      return sum(sel),pc,cnt[n],coord[n]
    else:
      n=np.argmax(cnt)
      return 0,pc,cnt[n],coord[n]
  else:
    return 0,pc,0,0

def pCropY(pc,y0,yh):
  cy=RT[1,3]/2

def cb_ps(msg):
  global Scene,SceneP,do1cema
  Scene=np.reshape(msg.data,(-1,3))
  SceneP=Scene
  pub_ps.publish(np2F(Scene))
  try:
    Param.update(rospy.get_param("/prepro"))
  except Exception as e:
    print("get_param exception:",e.args)
  print("prepro::cb_ps",Param)
  report={}
  if Param["enable"]==0:
    report['probables']=1
    report['volume']=0
    report['margin']=100
    msg=String()
    msg.data=str(report)
    pub_str.publish(msg)
    if do1cema>0:
      do1cema=0
      rospy.Timer(rospy.Duration(0.1),lambda ev: pub_done1.publish(mTrue),oneshot=True)
    return
  P0=getRT("world",Config["axis_frame_ids"][0])[0:3,3].ravel()
  Px=getRT("world",Config["axis_frame_ids"][1])[0:3,3].ravel()
  wTu,xlen=pFrame(P0,Px)
  print("Bucket",wTu)
  wTc=getRT("world","camera/capture0")
  uTc=np.linalg.inv(wTu).dot(wTc)
  cTu=np.linalg.inv(uTc)
  print("Scene",len(Scene))
  uscn=pTr(uTc,Scene)
  lx=np.linalg.norm(Px-P0)
  pbn,uscn0,cnt0,cx0=pScanX(uscn,lx,Param["box0_width"],Param["box0_depth"],Param["box0_points"])  #Scan PC peak of F-end of workpiece
  print("prepro",pbn,len(uscn0),cnt0,cx0)
  report["probables"]=pbn
  if pbn>0: #probable point clusters
    report["volume"]=cnt0
    Pc=np.ravel(uTc[0:3,3])
    margin=cx0 if cx0<xlen/2 else xlen-cx0
    report["margin"]=margin
    print("margin",report["margin"])
    uscn1=uscn[ np.ravel(np.abs(uscn[:,0]-cx0)<Param["box0_crop"]/2) ]
    SceneP=pTr(cTu,uscn1)
    print("Scene prepro",len(SceneP))
  else:
    SceneP=None
  if do1cema>0:
    do1cema=0
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_done1.publish(mTrue),oneshot=True)
  msg=String()
  msg.data=str(report)
  pub_str.publish(msg)


def cb_do1(msg):
  global do1cema
  do1cema=do1cema+1

def cb_do2(msg):
  global SceneP
  if Param["enable"]==0:
    pub_ps.publish(np2F(Scene))
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_done2.publish(mTrue),oneshot=True)
    return
  if SceneP is not None:
    pub_ps.publish(np2F(SceneP))
  rospy.Timer(rospy.Duration(0.1),lambda ev: pub_done2.publish(mTrue),oneshot=True)

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
pub_ps=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_str=rospy.Publisher("/report",String,queue_size=1)
pub_done1=rospy.Publisher("~done1",Bool,queue_size=1)
pub_done2=rospy.Publisher("~done2",Bool,queue_size=1)
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
