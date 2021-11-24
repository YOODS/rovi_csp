# 点群のクロップ処理

## クロップ範囲の取得  
### クロップ位置
バケットの両端のクロップ位置は以下の名前でTFに登録されています。
- bucket_edge0
- bucket_edge1

### クロップ位置の取得  
撮影位置はcamera/capure0という名前でTFに登録されています。点群はこの座標系基準なので、同座標系にてクロップ位置を取得します。

### 実装  
座標を得るには、tfパッケージを使います。以下のようにパッケージをimportします。
~~~
import tf
import tf2_ros
~~~
初期化処理にて以下のようにBufferとListenerを生成します。
~~~
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
~~~
これにてBufferにTF状態が常時更新され、任意の座標系をいつでも得ることができます。以下は撮影座標系(camera/capture0)から見たクロップ位置(bucket_edge0)を取得します。
~~~
ts=tfBuffer.lookup_transform('camera/capture0','bucket_edge0',rospy.Time())
~~~
結果はTransformStamped型で返されます。例えばY座標であれば以下のように参照できます。
~~~
ts.transform.translation.y
~~~

## クロップ方法  
上記の処理にてクロップ両端のY座標(y1,y2 ただしy1<y2)が得られているとし、この範囲外の点を除外する方法を示します。

### <y1の削除  
点群はpcdという[N,3]のndarray形式になっています。この名前をpcdとします。pcdのY座標のみ取り出す表記はpcd[:,1]ですが、これとy1を比較します。
~~~
gt=pcd[:,1]>y1
~~~ 
gtはpcdと長さが同じBool型配列になります。Bool型配列を配列のインデックスに与えると、Trueの箇所のみ残ります。
~~~
pcd=pcd[gt]
~~~
同様に >y2の点を除外します。


