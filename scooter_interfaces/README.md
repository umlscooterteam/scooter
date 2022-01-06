# scooter_interfaces package

services
---
```
HoldingObject.srv

---
uint8 BASKET=0
uint8 PLACE=1
uint8 transition
```
```
Pick.srv

sensor_msgs/PointCloud2 sample_points
sensor_msgs/PointCloud2 stitched_cloud
int32[] center
---
bool success
```
```
PickSelection.srv

sensor_msgs/PointCloud2 stitched_cloud
int32[] center
---
uint8 YES=0
uint8 NO=1
uint8 transition
int32[] center
sensor_msgs/PointCloud2 sample_points
sensor_msgs/PointCloud2 stitched_cloud
```
```
PickSelectionConfirm.srv

---
uint8 PICK=0
uint8 BACK=1
uint8 transition
int32[] center
sensor_msgs/PointCloud2 stitched_cloud

```

actions
---
```
GetCloud.action

---
sensor_msgs/PointCloud2 stitched_cloud
bool success
---
uint8 percentage_complete
```
```
Segmentation.action

sensor_msgs/PointCloud2 stitched_cloud
int32[] center
---
sensor_msgs/PointCloud2 sample_points
sensor_msgs/PointCloud2 stitched_cloud
---
uint8 percentage_complete
```