#pragma once

/*
 * 发射机构控制器入口。
 *
 * 根据发射状态机当前状态，选择速度控制、角度控制或卡弹处理模式，
 * 并将结果写入发射机构电机输出。
 */
void RunLaunchController();

/* 卡弹检测结果供控制任务事件检测使用。 */
bool is_jamming();
