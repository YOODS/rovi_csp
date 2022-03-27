#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import time
from scipy.spatial.transform import Rotation as R
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
  "mesh":10,
  "box0_width":60,
  "box0_depth":40,
  "box0_points":500,
  "box0_low":100,
  "box0_crop":200,
  "bucket_width":870,
  "bucket_yclip":100,
  "ss_dist":500,
  "ss_rx":0,
  "ss_ry":30
}
Config={
  "axis_frame_id":"prepro"
}

Tm_do1=0
Tm_done1=0
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

def pTr(RT,pc):
  return np.dot(RT[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def pubPose(coordx,coordy,coordz):
  peaks=PoseArray()
  peaks.header.frame_id=Config['axis_frame_id']
  for x,y,z in zip(coordx,coordy,coordz):
    p=Pose()
    p.position.x=x
    p.position.y=y
    p.position.z=z
    p.orientation.x=0
    p.orientation.y=0
    p.orientation.z=-0.707
    p.orientation.w=0.707
    peaks.poses.append(p)
  pub_peaks.publish(peaks)

def pMap(pc,dx,dz):
  pitch=Param["mesh"]
  hx0=np.min(pc.T[0])
  hx1=np.max(pc.T[0])
  hz0=np.min(pc.T[2])
  hz1=np.max(pc.T[2])
  xdiv=np.arange(hx0,hx1,pitch)
  zdiv=np.arange(hz0,hz1,pitch)
  nlst=[]
  ylst=[]
  for x in xdiv:
    pc1=pc[ np.ravel(np.abs(pc[:,0]-x)<dx) ]
    for z in zdiv:
      pc2=pc1[ np.ravel(np.abs(pc1[:,2]-z)<dz) ]
      n2=len(pc2)
      nlst.append(n2)
      if n2>0: ylst.append(np.mean(pc2[:,1]))
      else: ylst.append(0)
  nm=(len(xdiv),len(zdiv))
  return xdiv,zdiv,np.array(nlst).reshape(nm),np.array(ylst).reshape(nm)

def pCollectPeakX(pc,dx,dz):
  xdiv,zdiv,nmap,ymap=pMap(pc,dx,dz)
  ncod=[]
  xcod=[]
  ycod=[]
  zcod=[]
  if len(xdiv)<3 or len(zdiv)<3: return ncod,xcod,ycod,zcod
  for i in np.arange(1,len(xdiv)-1):
    for j in np.arange(1,len(zdiv)-1):
      n=nmap[i,j]
      f=np.ravel(nmap[i-1:i+2,j-1:j+2]>=n)
      if sum(f)==1 and n>Param["box0_low"]:
        ncod.append(n)
        xcod.append(xdiv[i])
        ycod.append(ymap[i,j])
        zcod.append(zdiv[j])
  return np.array(ncod),np.array(xcod),np.array(ycod),np.array(zcod)

def pSelect1(coordx,coordy,coordz):
  l=Param["bucket_width"]
  xc=uTc[0,3]
  yc=uTc[1,3]
  zc=uTc[2,3]-Param["ss_dist"]
  R1=R.from_euler('XY',[Param['ss_rx'],Param['ss_ry']],degrees=True).as_matrix()
  R2=R.from_euler('XY',[Param['ss_rx'],-Param['ss_ry']],degrees=True).as_matrix()
  if xc<l/2:
    n1vec=np.ravel(R1.dot(np.array([[0],[0],[1]])))
    n2vec=np.ravel(R2.dot(np.array([[0],[0],[1]])))
  else:
    n1vec=np.ravel(R2.dot(np.array([[0],[0],[1]])))
    n2vec=np.ravel(R1.dot(np.array([[0],[0],[1]])))
  d1=-np.inner(n1vec,np.array([xc,yc,zc]))
  d2=-np.inner(n2vec,np.array([l-xc,yc,zc]))
  s1=np.dot(n1vec,np.array([coordx,coordy,coordz]))+d1
  s2=np.dot(n2vec,np.array([coordx,coordy,coordz]))+d2
  print('s1',s1)
  print('s2',s2)
  s0=np.min(np.array([s1,s2]),axis=0)
  print('s0',s0)
  n=np.argmax(s0)
  if s0[n]>0: return n
  else: return -n-1

def pScanX(pc):
  h=Param["bucket_width"]
  wid=Param["box0_width"]
  dep=Param["box0_depth"]
  thres=Param["box0_points"]
  pc=pc[ np.ravel(pc[:,1]<Param["bucket_yclip"]) ] # Y<clip
  if len(pc)<thres: return 0,0,0,0,0

  cnt,coordx,coordy,coordz=pCollectPeakX(pc,wid/2,dep/2)  #Collects PC peak
  print("scanX",cnt,coordx,coordy,coordz)
  if len(coordx)>0:
    sel=cnt>thres
    if sum(sel)>0:
      cnt=cnt[sel]
      coordx=coordx[sel]
      coordy=coordy[sel]
      coordz=coordz[sel]
      pubPose(coordx,coordy,coordz)
      n=pSelect1(coordx,coordy,coordz)
      if n>=0:
        return sum(sel),cnt[n],coordx[n],coordy[n],coordz[n]
      else:
        return 0,cnt[-n-1],coordx[-n-1],coordy[-n-1],coordz[-n-1]
    else:
      n=np.argmax(cnt)
      pubPose([coordx[n]],[coordy[n]],[coordz[n]])
      return 0,cnt[n],coordx[n],coordy[n],coordz[n]
  else:
    pub_peaks.publish(peaks)
    return 0,0,0,0,0

def done1(report):
  global Scene,SceneP,Tm_done1,Tm_do1
  if len(report)>0:
    msg=String()
    msg.data=str(report)
    pub_str.publish(msg)
  t=time.time()
  if t-Tm_do1<5:
    Tm_do1=0
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_done1.publish(mTrue),oneshot=True)
  else:
    Tm_done1=t

def cb_ps(msg):
  global Scene,SceneP,SceneN,ParamN,uTc
  try:
    Param.update(rospy.get_param("/prepro"))
  except Exception as e:
    print("get_param exception:",e.args)
  print('prepro cb_ps',time.time()-Tm_do1)
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
  uTc=getRT(Config["axis_frame_id"],"camera/capture0")
  cTu=np.linalg.inv(uTc)
  uscn=pTr(uTc,Scene)
  pbn,cnt0,cx0,cy0,cz0=pScanX(uscn)  #Scan PC peak of F-end of workpiece
  report={}
  report["probables"]=pbn
  report["prob_v"]=cnt0
  report["prob_x"]=cx0
  report["prob_z"]=uTc[2,3]-cz0
  if pbn>0: #probable point clusters
    xlen=Param["bucket_width"]
    margin=cx0 if cx0<xlen/2 else xlen-cx0
    report["prob_m"]=margin
    uscn1=uscn[ np.ravel(np.abs(uscn[:,0]-cx0)<Param["box0_crop"]/2) ]
    SceneP=pTr(cTu,uscn1)
    print("cb_ps done",len(SceneP))
  else:
    print("cb_ps done None")
    SceneP=Pnul()
  done1(report)

def cb_do1(msg):
  global Tm_do1
  t=time.time()
  print('prepro do1',t-Tm_done1)
  if t-Tm_done1<1:
    pub_done1.publish(mTrue)
  else:
    Tm_do1=t

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
