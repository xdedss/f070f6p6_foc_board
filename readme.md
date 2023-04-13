
### 用于3205电机的FOC控制模块

此代码仓库是运行在下面硬件上的一个简单的FOC控制demo，还没有实现I2C功能

硬件：https://oshwhub.com/xdedss/motor3205_driver_board

### 注意事项

根据安装角度的微弱差异，需要调整磁编码器角度到电角度的转换关系式。在 main.c 中199行左右：

``` c
    elecAngle = -(mag_r - 0.60) * 7 * PI * 2; 
```

其中0.60可能会因安装角度不同而需要修正。如何知道修正值？方法是让电机自己转到0电角度，然后读取此时磁编码器的值。

While循环里只留两行代码

``` c
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      readMagSsi();
      UVW_120deg(0, 1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

```

进入调试，不要给电机施加任何外力，待电机角度稳定后（此时用手转电机应该会感到一股力量使其回到特定的角度）查看全局变量mag_r的值，用这个值替换掉0.60即可

另外该关系式的正负号可能需要根据磁编码器安装的正反来调整。安装在BTM面的话就和上面代码一致，安装在TOP面则需要去掉负号。

![image](img.jpg)

