### 修改内容

- 整理、重构大部分代码。
- 将原有 `findone` 重构为 `utils`。
- 去掉各节点的 `imshow`，统一使用 `monitor` 节点订阅并显示图像，后期可以用主从机。（实测帧率提升 15-20）
- 修改 `open_camera` 逻辑，支持断线重连。
- 为了加入对视频的支持，将原有 `open_camera` 拆成 `open_camera` 和 `image_process` 两个节点，后者只负责图像的预处理。
- 增加节点 `open_video`，支持使用视频测试识别效果。

### 现存问题

- 在测试视频等复杂环境中的识别效果不好。

### TODO

- 修改识别算法，提高准确率和识别速度。
- 加卡尔曼滤波。
- 写 `launch`。

### 识别测试

```bash
rosrun receive open_video
rosrun receive image_process
rosrun receive contour_retrival
```