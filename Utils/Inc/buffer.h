/*
 * Buffer module interface
 * 默认实现：内部使用长度为 10 的 float 数组存储数据。
 */

#ifndef BUFFER_H_
#define BUFFER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

/* 默认内部缓冲区容量（实现文件中为固定 4） */
#define BUFFER_DEFAULT_CAPACITY 10U

/* 最大内部缓冲区容量（实现文件中为固定 20） */
#define BUFFER_MAX_CAPACITY 20U

    /*
 * 初始化缓冲区
 * length: 期望容量（会被限制在 BUFFER_DEFAULT_CAPACITY 以内）
 * 作用：设置有效容量并清空缓冲区
 */
    void Buffer_Init(size_t length, size_t prescaler);

    /*
 * 写入数据到缓冲区指定位置
 * value: 要写入的数据
 * index: 目标下标（基于 0），越界将被忽略
 */
    void Buffer_Put(float value, size_t index);

    /*
 * 发送缓冲区数据（空实现，占位，留给用户编写）
 */
    void Buffer_Send(void);

#ifdef __cplusplus
}
#endif

#endif /* BUFFER_H_ */
