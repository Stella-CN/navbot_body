#ifndef NAVBOT_APP_TASKS_H
#define NAVBOT_APP_TASKS_H

#ifdef __cplusplus
extern "C"
{
#endif

    void Navbot_ControlTask(void *argument);
    void Navbot_CommTask(void *argument);
    void Navbot_SensorTask(void *argument);
    void Navbot_MonitorTask(void *argument);

#ifdef __cplusplus
}
#endif
#endif
