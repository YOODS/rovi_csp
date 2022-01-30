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
  "box0_width":60,
  "box0_depth":40,
  "box0_points":700,
  "box0_crop":200,
  "post_margin":100
}
Config={
  "axis_frame_ids":["bucket_post0","bucket_post1"]
}

def cb_redraw(msg):
  pub_ps.publish(np2F(Scene))

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
  dmax=0
  cnt=[]
  cod=[]
  for x in np.arange(0,hx,pitch):
    pc1=pc[ np.ravel(np.abs(pc[:,0]-x)<dx) ]
    cmax=0
    for z in np.arange(0,hz,pitch):
      n=len(pc1[ np.ravel(np.abs(pc1[:,2]-z)<dz) ])
      if cmax<n: cmax=n
    if dmax>cmax:
      cnt.append(dmax)
      cod.append(x-pitch)
    dmax=cmax
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
      return pc,cnt[n],coord[n]
    else:
      n=np.argmax(cnt)
      return pc,cnt[n],coord[n]
  else:
    return pc,None,None

def pCropY(pc,y0,yh):
  cy=RT[1,3]/2


def cb_ps(msg):
  global Scene
  Scene=np.reshape(msg.data,(-1,3))
  pub_ps.publish(np2F(Scene))
  return

def cb_solve(msg):
  global Scene
  try:
    Param.update(rospy.get_param("/prepro"))
  except Exception as e:
    print("get_param exception:",e.args)
  print("prepro param",Param)
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
  uscn0,cnt0,cx0=pScanX(uscn,lx,Param["box0_width"],Param["box0_depth"],Param["box0_points"])  #Scan PC peak of F-end of workpiece
  print("count0",len(uscn0),cnt0,cx0)
  report={}
  if cnt0 is not None:
    if cnt0>Param["box0_points"]:
      report["volume"]=(cnt0,0)
      Pc=np.ravel(uTc[0:3,3])
      margin=-cx0 if cx0<Pc[0] else xlen-cx0
      report["margin"]=(margin,int(np.abs(margin)<Param["post_margin"]))
      print("margin",report["margin"])
      uscn1=uscn[ np.ravel(np.abs(uscn[:,0]-cx0)<Param["box0_crop"]/2) ]
      Scene=pTr(cTu,uscn1)
      cb_redraw(0)
      rospy.Timer(rospy.Duration(0.1),lambda ev: pub_thru.publish(mTrue),oneshot=True)
    else:
      report["volume"]=(cnt0,802)
      rospy.Timer(rospy.Duration(0.1),lambda ev: pub_cut.publish(mFalse),oneshot=True)
  else:
    report["volume"]=(0,801)
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_cut.publish(mFalse),oneshot=True)
  print("Scene",len(Scene))
  msg=String()
  msg.data=str(report)
  pub_str.publish(msg)

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
rospy.Subscriber("/request/solve",Bool,cb_solve)
rospy.Subscriber("/request/redraw",Bool,cb_redraw)
pub_ps=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_str=rospy.Publisher("/report",String,queue_size=1)
pub_thru=rospy.Publisher("~passthru",Bool,queue_size=1)
pub_cut=rospy.Publisher("~shortcut",Bool,queue_size=1)
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
