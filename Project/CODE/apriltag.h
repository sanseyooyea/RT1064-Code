/*
 * apriltag识别。赛道中线发现一坨黑色，并且不是斑马线即判定为apriltag
 */

#ifndef APRILTAG_H
#define APRILTAG_H

// 定义apriltag类型枚举
enum apriltag_type_e {
    APRILTAG_NONE = 0,  // 无黑斑apriltag
    APRILTAG_MAYBE,     // 识别到远距离黑斑，减速
    APRILTAG_FOUND,     // 识别到近距离黑斑，停车
    APRILTAG_LEAVE,     // 驶离apriltag中，编码器判断，防止其他标志的判断受到apriltag的影响
    APRILTAG_NUM,       // 枚举数量，用于数组大小定义
};

// 声明apriltag类型和时间变量
extern enum apriltag_type_e apriltag_type;
extern int apriltag_time;

// 定义apriltag类型名称数组
extern const char *apriltag_type_name[APRILTAG_NUM];

// 检查apriltag函数声明
void check_apriltag();

#endif // APRILTAG_H