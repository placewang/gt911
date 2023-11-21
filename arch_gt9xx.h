/*
* 电容触控屏驱动 GT911
*/
#ifndef __ARCH_GT9XX_H__
#define __ARCH_GT9XX_H__


typedef struct {
    int id;
    int x;
    int y;
    int w;  //pressure
    int touch;
} TouchEvent;

extern int arch_gt911_init();
extern int arch_gt9xx_get(TouchEvent * ev);


#endif  //__ARCH_GT9XX_H__
